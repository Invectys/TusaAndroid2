//
// Created by Artem on 11.05.2024.
//

#ifndef TUSA_ANDROID_CUBE_EXAMPLE_H
#define TUSA_ANDROID_CUBE_EXAMPLE_H


#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <GLES2/gl2.h>
#include "open_gl_interface.h"
#include "util/matrices.h"
#include "util/android_log.h"
#include "shader/shaders_bucket.h"

class CubeExample : public IOpenGl {
    void render() override;
    void onSurfaceChanged(int w, int h) override;
    void onSurfaceCreated(AAssetManager* assetManager) override;
    void noOpenGlContextInit(AAssetManager* assetManager, float scaleFactor, JNIEnv *env) override;
    void drag(float dx, float dy) override;
    void scale(float scaleFactor) override;
    void doubleTap() override;

public:
    CubeExample() {
        glVertexShader =
                "attribute vec4 vertexPosition;\n"
                "attribute vec3 vertexColour;\n"
                "varying vec3 fragColour;\n"
                "uniform mat4 projection;\n"
                "uniform mat4 modelView;\n"
                "void main()\n"
                "{\n"
                "    gl_Position = projection * modelView * vertexPosition;\n"
                "    fragColour = vertexColour;\n"
                "}\n";

        glFragmentShader =
                "precision mediump float;\n"
                "varying vec3 fragColour;\n"
                "void main()\n"
                "{\n"
                "    gl_FragColor = vec4(fragColour, 1.0);\n"
                "}\n";

        triangleVertices = new GLfloat[]{
                0.0f, 1.0f,
                -1.0f, -1.0f,
                1.0f, -1.0f
        };
        colour = new GLfloat[]{1.0f, 0.0f, 0.0f,
                    1.0f, 0.0f, 0.0f,
                    1.0f, 0.0f, 0.0f,
                    1.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 1.0f,
                    0.0f, 0.0f, 1.0f,
                    0.0f, 0.0f, 1.0f,
                    0.0f, 0.0f, 1.0f,
                    1.0f, 1.0f, 0.0f,
                    1.0f, 1.0f, 0.0f,
                    1.0f, 1.0f, 0.0f,
                    1.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 1.0f,
                    0.0f, 1.0f, 1.0f,
                    0.0f, 1.0f, 1.0f,
                    0.0f, 1.0f, 1.0f,
                    1.0f, 0.0f, 1.0f,
                    1.0f, 0.0f, 1.0f,
                    1.0f, 0.0f, 1.0f,
                    1.0f, 0.0f, 1.0f
        };

        cubeVertices = new GLfloat[]{-1.0f,  1.0f, -1.0f, /* Back. */
                          1.0f,  1.0f, -1.0f,
                          -1.0f, -1.0f, -1.0f,
                          1.0f, -1.0f, -1.0f,
                          -1.0f,  1.0f,  1.0f, /* Front. */
                          1.0f,  1.0f,  1.0f,
                          -1.0f, -1.0f,  1.0f,
                          1.0f, -1.0f,  1.0f,
                          -1.0f,  1.0f, -1.0f, /* Left. */
                          -1.0f, -1.0f, -1.0f,
                          -1.0f, -1.0f,  1.0f,
                          -1.0f,  1.0f,  1.0f,
                          1.0f,  1.0f, -1.0f, /* Right. */
                          1.0f, -1.0f, -1.0f,
                          1.0f, -1.0f,  1.0f,
                          1.0f,  1.0f,  1.0f,
                          -1.0f, -1.0f, -1.0f, /* Top. */
                          -1.0f, -1.0f,  1.0f,
                          1.0f, -1.0f,  1.0f,
                          1.0f, -1.0f, -1.0f,
                          -1.0f,  1.0f, -1.0f, /* Bottom. */
                          -1.0f,  1.0f,  1.0f,
                          1.0f,  1.0f,  1.0f,
                          1.0f,  1.0f, -1.0f
        };
        indices = new GLushort[]{0, 2, 3, 0, 1, 3, 4, 6, 7, 4, 5, 7, 8, 9, 10, 11, 8, 10, 12, 13, 14, 15, 12, 14, 16, 17, 18, 16, 19, 18, 20, 21, 22, 20, 23, 22};
    }

    static GLuint loadShader(GLenum shaderType, const char* shaderSource) {
        GLuint shader = glCreateShader(shaderType);
        if(shader) {
            glShaderSource(shader, 1, &shaderSource, NULL);
            glCompileShader(shader);
            GLint compiled = 0;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
            if(!compiled) {
                GLint infoLen = 0;
                glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
                if(infoLen) {
                    char* buf = (char*) malloc(infoLen);
                    if(buf) {
                        glGetShaderInfoLog(shader, infoLen, NULL, buf);
                        LOGE("Could not compile shader %d:\n%s\n", shaderType, buf);
                        free(buf);
                    }
                    glDeleteShader(shader);
                    shader = 0;
                }
            }
        }
        return shader;
    }

    static GLuint createProgram(const char* vertexSource, const char* fragmentSource) {
        GLuint vertexShader = loadShader(GL_VERTEX_SHADER, vertexSource);
        if(!vertexShader) {
            return 0;
        }
        GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER, fragmentSource);
        if(!fragmentShader) {
            return 0;
        }
        GLuint program = glCreateProgram();
        if(program) {
            glAttachShader(program, vertexShader);
            glAttachShader(program, fragmentShader);
            glLinkProgram(program);
            GLint linkStatus = GL_FALSE;
            glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
            if(linkStatus != GL_TRUE) {
                GLint bufLength = 0;
                glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufLength);
                if(bufLength) {
                    char* buf = (char *) malloc(bufLength);
                    if(buf) {
                        glGetProgramInfoLog(program, bufLength, NULL, buf);
                        LOGE("Could not link program:\n%s\n", buf);
                        free(buf);
                    }
                }
                glDeleteProgram(program);
                program = 0;
            }
        }
        return program;
    }


private:
    ShadersBucket shadersBucket = ShadersBucket();
    GLuint vPosition;
    GLuint vertexLocation;
    GLuint vertexColorLocation;
    GLuint projectionLocation;
    GLuint modelViewLocation;
    Matrix4 projectionMatrix;
    Matrix4 modelViewMatrix;
    float angle = 0;
    GLuint simpleTriangleProgram;

    std::string glVertexShader;
    std::string glFragmentShader;
    GLfloat *triangleVertices;
    GLfloat *colour;
    GLfloat *cubeVertices;
    GLushort *indices;
};



#endif //TUSA_ANDROID_CUBE_EXAMPLE_H
