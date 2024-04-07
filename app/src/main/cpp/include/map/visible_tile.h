//
// Created by Artem on 26.02.2024.
//

#ifndef TUSA_VISIBLE_TILE_H
#define TUSA_VISIBLE_TILE_H


#include "tile.h"

class VisibleTile {
public:
    VisibleTile(Tile* tile, int shiftX, int shiftY, float mapScaleFactor);
    VisibleTile();

    // Смещение в 3d пространстве относительно нуля по тайлам
    int shiftX;
    int shiftY;
    float mapScaleFactor;
    Tile* tile = nullptr;

    bool isEmpty() {
        return tile == nullptr;
    }
};


#endif //TUSA_VISIBLE_TILE_H
