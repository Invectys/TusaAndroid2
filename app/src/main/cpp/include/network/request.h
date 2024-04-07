//
// Created by Artem on 20.01.2024.
//

#ifndef TUSA_REQUEST_H
#define TUSA_REQUEST_H


#include "cache/cache.h"
#include "map/tile.h"
#include "style/style.h"

class Request {
public:
    Request(Cache* cache);
    Tile* loadVectorTile(int zoom, int x, int y);
private:
    Cache* cache;
};


#endif //TUSA_REQUEST_H
