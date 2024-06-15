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
    GLint getColorLocation() const {return u_color;}
    GLint getTileTextureLocation0() const {return u_tile_0;}
    GLint getTileTextureLocation1() const {return u_tile_1;}
    GLint getTileTextureLocation2() const {return u_tile_2;}
    GLint getTileTextureLocation3() const {return u_tile_3;}
    GLint getTileTextureCoordinates0() const {return a_tile_cords_0;}
    GLint getTileTextureCoordinates1() const {return a_tile_cords_1;}
    GLint getTileTextureCoordinates2() const {return a_tile_cords_2;}
    GLint getTileTextureCoordinates3() const {return a_tile_cords_3;}
    GLint getUnitSquareCoordinates() const {return a_unit_square_cords;}

    GLint getTileTextureCoordinate(short index) {
        if(index == 0) {
            return getTileTextureCoordinates0();
        }
        if(index == 1) {
            return getTileTextureCoordinates1();
        }
        if(index == 2) {
            return getTileTextureCoordinates2();
        }
        if(index == 3) {
            return getTileTextureCoordinates3();
        }
        throw std::invalid_argument("I have not planet tile texture coordinate with passed index");
    }

    GLint getTileTextureLocation(short index) {
        if(index == 0) {
            return getTileTextureLocation0();
        }
        if(index == 1) {
            return getTileTextureLocation1();
        }
        if(index == 2) {
            return getTileTextureLocation2();
        }
        if(index == 3) {
            return getTileTextureLocation3();
        }
        throw std::invalid_argument("I have not planet tile texture location with passed index");
    }

private:
    GLint a_pos;
    GLint a_tile_cords_0;
    GLint a_tile_cords_1;
    GLint a_tile_cords_2;
    GLint a_tile_cords_3;
    GLint a_unit_square_cords;
    GLint u_color;
    GLint u_tile_0;
    GLint u_tile_1;
    GLint u_tile_2;
    GLint u_tile_3;
};


#endif //TUSA_ANDROID_PLANET_SHADER_H
