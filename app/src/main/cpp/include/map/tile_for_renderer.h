//
// Created by Artem on 26.02.2024.
//

#ifndef TUSA_VISIBLE_TILE_H
#define TUSA_VISIBLE_TILE_H


#include "tile.h"

class TileForRenderer {
public:
    TileForRenderer(Tile* tile, int shiftX, int shiftY, int tileX, int tileY, int tileZ, float mapScaleFactor);
    TileForRenderer();

    // Смещение в 3d пространстве относительно нуля по тайлам
    // (Это координаты тайла без нормализации)
    int shiftX;
    int shiftY;

    int tileX;
    int tileY;
    int tileZ;

    short zDeltaFlag = 0;

    float mapScaleFactor;
    Tile* tile = nullptr;

    bool cover(TileForRenderer otherTile) {
        int otherX = otherTile.tileX;
        int otherY = otherTile.tileY;
        int otherZ = otherTile.tileZ;

        int extent = pow(2, 19 - tileZ);
        int startX = tileX * extent;
        int endX = startX + extent;
        int startY = tileY * extent;
        int endY = startY + extent;

        int otherExtent = pow(2, 19 - otherZ);
        int otherStartX = otherX * otherExtent;
        int otherEndX = otherStartX + otherExtent;
        int otherStartY = otherY * otherExtent;
        int otherEndY = otherStartY + otherExtent;

        bool xCovered = startX <= otherStartX && otherEndX <= endX;
        bool yCovered = startY <= otherStartY && otherEndY <= endY;
        return xCovered && yCovered;
    }

    void clear() {
        tile = nullptr;
    }

    bool isEmpty() {
        return tile == nullptr;
    }
};


#endif //TUSA_VISIBLE_TILE_H
