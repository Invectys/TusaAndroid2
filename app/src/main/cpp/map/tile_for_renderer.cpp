//
// Created by Artem on 26.02.2024.
//

#include "map/tile_for_renderer.h"

TileForRenderer::TileForRenderer(Tile *tile, int shiftX, int shiftY,
                                 int tileX, int tileY, int tileZ, float mapScaleFactor)
    : tile(tile), shiftX(shiftX), shiftY(shiftY),
      tileX(tileX), tileY(tileY), tileZ(tileZ), mapScaleFactor(mapScaleFactor) {}

TileForRenderer::TileForRenderer() {

}
