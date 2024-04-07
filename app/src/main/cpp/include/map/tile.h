//
// Created by Artem on 08.02.2024.
//

#ifndef TUSA_TILE_H
#define TUSA_TILE_H

#include <string>
#include <boost/variant.hpp>
#include "geometry/geometry.h"
#include <forward_list>
#include <vector_tile.hpp>
#include "util/feature_geometry_block.h"
#include "style/style.h"
#include "renderer/map_draw_call.h"

using PolygonCoord = float;
using PolygonIndic = unsigned int;
using PolygonPoint = std::array<PolygonCoord, 2>;

using LineCoord = float;
using LineIndic = unsigned int;

class Tile {
public:
    Tile(int zoom, int x, int y, vtzero::vector_tile *tile);

    static std::string makeKey(int zoom, int x, int y);
    int getZ() {return zoom;}
    int getX() {return x;}
    int getY() {return y;}

    Style style = Style();
    Geometry<LineCoord, LineIndic> *resultLines = new Geometry<LineCoord, LineIndic>[Style::maxGeometryHeaps];
    Geometry<PolygonCoord, PolygonIndic> *resultPolygons = new Geometry<PolygonCoord, PolygonIndic>[Style::maxGeometryHeaps];

    std::vector<MapDrawCall> drawCalls = {};

    ~Tile() {
        delete[] resultLines;
        delete[] resultPolygons;
    }
private:

    int zoom;
    int x;
    int y;

    void parseMethod1(int zoom, int x, int y, vtzero::vector_tile *tile);
};


#endif //TUSA_TILE_H
