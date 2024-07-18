//
// Created by Artem on 11.05.2024.
//

#include "shader/planet_shader.h"

PlanetShader::PlanetShader(AAssetManager *assetManager, const char *vertexShaderName,
                           const char *fragmentShaderName) : Shader(assetManager, vertexShaderName,
                                                                    fragmentShaderName) {
    if (!valid) {
        fprintf(stderr, "invalid symbol shader\n");
        return;
    }

    u_matrix = glGetUniformLocation(program, "u_matrix");
    a_pos = glGetAttribLocation(program, "a_vertexPosition");
    a_unit_square_cords = glGetAttribLocation(program, "a_unit_square_cords");
    u_tile_0 = glGetUniformLocation(program, "u_tile_0");
}
