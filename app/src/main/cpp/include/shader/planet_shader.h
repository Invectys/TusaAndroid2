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

    GLint getTileTopLeftXPtr() const {return u_topLeftX;}
    GLint getTileTopLeftYPtr() const {return u_topLeftY;}

    GLint getTileBottomRightXPtr() const {return u_bottomRightX;}
    GLint getTileBottomRightYPtr() const {return u_bottomRightY;}

    GLint getShiftX() const {return u_shiftX;}
    GLint getShiftY() const {return u_shiftY;}

    GLint getZoom() const {return u_zoom;}

    GLint getStartX() const {return u_startX;}
    GLint getEndX() const {return u_endX;}
    GLint getStartY() const {return u_startY;}
    GLint getEndY() const {return u_endY;}

private:
    GLint a_pos;
    GLint a_unit_square_cords;
    GLint u_tile_0;
    GLint u_topLeftX;
    GLint u_topLeftY;
    GLint u_bottomRightX;
    GLint u_bottomRightY;

    GLint  u_startX;
    GLint  u_endX;

    GLint  u_startY;
    GLint  u_endY;

    GLint  u_shiftX;
    GLint  u_shiftY;
    GLint  u_zoom;
};


#endif //TUSA_ANDROID_PLANET_SHADER_H
