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

    bool needToInsertPassed = false;

    std::string toString() {
        return "(" + std::to_string(tileX) + ", " + std::to_string(tileY) + ", " + std::to_string(tileZ) + ")";
    }

    bool is(TileCords otherTile) {
        return otherTile.tileX == tileX && otherTile.tileZ == tileZ && otherTile.tileY == tileY;
    }

    bool cover(TileCords otherTile) {
        if (tileZ > otherTile.tileZ) {
            return false;
        }

        // Коэффициент масштаба между уровнями зума
        int scale = 1 << (otherTile.tileZ - tileZ);

        // Преобразуем координаты тайла1 на уровень зума тайла2
        int transformed_x1 = tileX * scale;
        int transformed_y1 = tileY * scale;

        // Проверяем, покрывает ли тайл1 тайл2
        return (transformed_x1 <= otherTile.tileX && otherTile.tileX < transformed_x1 + scale) &&
               (transformed_y1 <= otherTile.tileY && otherTile.tileY < transformed_y1 + scale);

    }
};


#endif //TUSA_ANDROID_TILE_CORDS_H
