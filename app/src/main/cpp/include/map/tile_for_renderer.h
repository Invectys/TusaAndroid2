//
// Created by Artem on 26.02.2024.
//

#ifndef TUSA_VISIBLE_TILE_H
#define TUSA_VISIBLE_TILE_H


#include "tile.h"
#include "renderer/tile_cords.h"

class TileForRenderer {
public:
    TileForRenderer(Tile* tile, int shiftX, int shiftY, int tileX, int tileY, int tileZ, float mapScaleFactor,
                    int rPosX, int rPosY, short xNorm, short yNorm, TileCords tileCords);
    TileForRenderer();

    // Смещение в 3d пространстве относительно нуля по тайлам
    // (Это координаты тайла без нормализации)
    int shiftX;
    int shiftY;

    int tileX;
    int tileY;
    int tileZ;
    int rPosX;
    int rPosY;

    int xNorm;
    int yNorm;

    short zDeltaFlag = 0;

    float mapScaleFactor;
    bool notActual = false;
    Tile* tile = nullptr;
    TileCords tileCords;

    std::string toString() {
        return "(" + std::to_string(tileX) + ", " + std::to_string(tileY) + ", " + std::to_string(tileZ) + ")"
        + " rPos(" + std::to_string(rPosX) + ", " + std::to_string(rPosY) + ")";
    }

    void clear() {
        tile = nullptr;
    }

    bool isEmpty() {
        return tile == nullptr;
    }
};


#endif //TUSA_VISIBLE_TILE_H
