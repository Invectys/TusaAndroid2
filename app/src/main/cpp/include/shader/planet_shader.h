//
// Created by Artem on 11.05.2024.
//

#ifndef TUSA_ANDROID_PLANET_SHADER_H
#define TUSA_ANDROID_PLANET_SHADER_H


#include "shader.h"

class PlanetShader : public Shader {
public:
    PlanetShader(AAssetManager* assetManager, const char* vertexShaderName, const char* fragmentShaderName);

    GLint getPosLocation() const {return a_pos;}
    GLint getTileTextureLocation0() const {return u_tile_0;}
    GLint getUnitSquareCoordinates() const {return a_unit_square_cords;}

private:
    GLint a_pos;
    GLint a_unit_square_cords;
    GLint u_tile_0;
};


#endif //TUSA_ANDROID_PLANET_SHADER_H
