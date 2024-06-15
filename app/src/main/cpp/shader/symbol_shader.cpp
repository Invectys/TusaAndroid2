//
// Created by Artem on 30.03.2024.
//

#include "shader/symbol_shader.h"

SymbolShader::SymbolShader(AAssetManager *assetManager,
                           const char *vertexShaderName,
                           const char *fragmentShaderName) :
                           Shader(assetManager, vertexShaderName, fragmentShaderName) {

    if (!valid) {
        fprintf(stderr, "invalid symbol shader\n");
        return;
    }
    a_pos = glGetAttribLocation(program, "vertexPosition");
    a_textureCord = glGetAttribLocation(program, "a_textureCord");
    u_matrix = glGetUniformLocation(program, "u_matrix");
    u_color = glGetUniformLocation(program, "u_color");
    u_text = glGetUniformLocation(program, "u_tile_0");
}
