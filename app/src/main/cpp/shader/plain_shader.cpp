//
// Created by Artem on 17.12.2023.
//

#include <GLES2/gl2.h>
#include "shader/plain_shader.h"
#include "shader/shaders_bucket.h"

PlainShader::PlainShader(AAssetManager* assetManager, const char* vertexShaderName, const char* fragmentShaderName)
    : Shader(assetManager, vertexShaderName, fragmentShaderName) {
    if (!valid) {
        fprintf(stderr, "invalid plain shader\n");
        return;
    }
    a_pos = glGetAttribLocation(program, "vertexPosition");
    u_matrix = glGetUniformLocation(program, "u_matrix");
    u_color = glGetUniformLocation(program, "u_color");
    u_pointSize = glGetUniformLocation(program, "u_pointSize");
}

