//
// Created by Artem on 07.02.2024.
//

#ifndef TUSA_TILES_STORAGE_H
#define TUSA_TILES_STORAGE_H

#include "network/get_tile_request.h"
#include "tile.h"
#include "style/style.h"

class TilesStorage {
public:
    TilesStorage(Cache* cache, GetTileRequest* request);
    Tile* getTile(int zoom, int x, int y, GetTileRequest* getTileRequest);
    bool existInMemory(int zoom, int x, int y);

    ~TilesStorage();
private:
    std::map<std::string, Tile*> cacheTiles = {};
    std::shared_ptr<Style> style;
    Cache* cache;
    GetTileRequest* request;
};


#endif //TUSA_TILES_STORAGE_H
