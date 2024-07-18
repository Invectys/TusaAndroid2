//
// Created by Artem on 09.04.2024.
//

#ifndef TUSA_ANDROID_TILE_CORDS_H
#define TUSA_ANDROID_TILE_CORDS_H

#include <iostream>

class TileCords {
public:
    int tileXShift;
    int tileYShift;
    int tileX;
    int tileY;
    int tileZ;
    short xNormalized;
    short yNormalized;
    int renderPosX = 0;
    int renderPosY = 0;
    float xComponent = 0;
    float yComponent = 0;
    float longitudeRad = 0;
    float latitudeRad = 0;
    bool hasLatitudeAndLongitudeRad = false;

    std::string toString() {
        return "(" + std::to_string(tileX) + ", " + std::to_string(tileY) + ", " + std::to_string(tileZ) + ")";
    }
};


#endif //TUSA_ANDROID_TILE_CORDS_H
