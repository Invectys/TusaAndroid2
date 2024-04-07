//
// Created by Artem on 26.02.2024.
//

#include "map/visible_tile.h"

VisibleTile::VisibleTile(Tile *tile, int shiftX, int shiftY, float mapScaleFactor)
    : tile(tile), shiftX(shiftX), shiftY(shiftY), mapScaleFactor(mapScaleFactor) {

}

VisibleTile::VisibleTile() {

}
