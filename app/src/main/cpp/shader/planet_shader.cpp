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

    u_topLeftX = glGetUniformLocation(program, "xTopLeft");
    u_topLeftY = glGetUniformLocation(program, "yTopLeft");
    u_bottomRightX = glGetUniformLocation(program, "xBottomRight");
    u_bottomRightY = glGetUniformLocation(program, "yBottomRight");

    u_zoom = glGetUniformLocation(program, "u_zoom");

    u_shiftX = glGetUniformLocation(program, "u_shiftX");
    u_shiftY = glGetUniformLocation(program, "u_shiftY");

    u_startX = glGetUniformLocation(program, "u_startX");
    u_endX = glGetUniformLocation(program, "u_endX");
    u_startY = glGetUniformLocation(program, "u_startY");
    u_endY = glGetUniformLocation(program, "u_endY");
}
