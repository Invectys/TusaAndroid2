//
// Created by Artem on 17.12.2023.
//

#include <GLES2/gl2.h>
#include "shader/shader.h"
#include "util/android_log.h"

Shader::Shader(AAssetManager* assetManager, const char* vertexShaderName, const char* fragmentShaderName)
    : valid(false), program(0) {

    AAsset* assetVertex = AAssetManager_open(assetManager, vertexShaderName, AASSET_MODE_BUFFER);
    long sizeVertex = AAsset_getLength(assetVertex);
    char *bufferVertex = (char*) malloc(sizeof(char)*sizeVertex);
    AAsset_read(assetVertex, bufferVertex, sizeVertex);
    AAsset_close(assetVertex);

    GLuint vertShader;
    if (!compileShader(&vertShader, GL_VERTEX_SHADER, bufferVertex, sizeVertex)) {
        return;
    }

    AAsset* assetFragment = AAssetManager_open(assetManager, fragmentShaderName, AASSET_MODE_BUFFER);
    long sizeFragment = AAsset_getLength(assetFragment);
    char *bufferFragment = (char*) malloc(sizeof(char)*sizeFragment);
    AAsset_read(assetFragment, bufferFragment, sizeFragment);
    AAsset_close(assetFragment);

    GLuint fragShader;
    if (!compileShader(&fragShader, GL_FRAGMENT_SHADER, bufferFragment, sizeFragment)) {
        return;
    }

    program = glCreateProgram();
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);

    {
        GLint status;
        glLinkProgram(program);
        glGetProgramiv(program, GL_LINK_STATUS, &status);
        if (status == 0) {
            fprintf(stderr, "Program failed to link\n");
            glDeleteShader(vertShader);
            vertShader = 0;
            glDeleteShader(fragShader);
            fragShader = 0;
            glDeleteProgram(program);
            program = 0;
            return;
        }
    }

    {
        GLint status;
        glValidateProgram(program);
        glGetProgramiv(program, GL_VALIDATE_STATUS, &status);
        if (status == 0) {
            glDeleteShader(vertShader);
            vertShader = 0;
            glDeleteShader(fragShader);
            fragShader = 0;
            glDeleteProgram(program);
            program = 0;
        }
    }

    // Remove the compiled shaders; they are now part of the program.
    glDetachShader(program, vertShader);
    glDeleteShader(vertShader);
    glDetachShader(program, fragShader);
    glDeleteShader(fragShader);

    valid = true;
}

bool Shader::compileShader(GLuint* shader, GLenum shaderType, const char *source, GLint length) {
    GLint status;
    *shader = glCreateShader(shaderType);
    glShaderSource(*shader, 1, &source, &length);
    glCompileShader(*shader);
    glGetShaderiv(*shader, GL_COMPILE_STATUS, &status);
    if (status == 0) {
        GLint infoLen = 0;
        glGetShaderiv(*shader, GL_INFO_LOG_LENGTH, &infoLen);
        if(infoLen) {
            char* buf = (char*) malloc(infoLen);
            if(buf) {
                glGetShaderInfoLog(*shader, infoLen, NULL, buf);
                LOGE("Could not compile shader %d:\n%s\n", shaderType, buf);
                free(buf);
            }
            glDeleteShader(*shader);
            shader = 0;
        }
        return false;
    }

    return true;
}

Shader::~Shader() {
    if (program) {
        glDeleteProgram(program);
        program = 0;
        valid = false;
    }
}
