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

    u_color = glGetUniformLocation(program, "color");
    u_matrix = glGetUniformLocation(program, "u_matrix");
    a_pos = glGetAttribLocation(program, "a_vertexPosition");
    a_tile_cords_0 = glGetAttribLocation(program, "a_tile_cords_0");
    a_tile_cords_1 = glGetAttribLocation(program, "a_tile_cords_1");
    a_tile_cords_2 = glGetAttribLocation(program, "a_tile_cords_2");
    a_tile_cords_3 = glGetAttribLocation(program, "a_tile_cords_3");
    a_unit_square_cords = glGetAttribLocation(program, "a_unit_square_cords");
    u_tile_0 = glGetUniformLocation(program, "u_tile_0");
    u_tile_1 = glGetUniformLocation(program, "u_tile_1");
    u_tile_2 = glGetUniformLocation(program, "u_tile_2");
    u_tile_3 = glGetUniformLocation(program, "u_tile_3");
}
