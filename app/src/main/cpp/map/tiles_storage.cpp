//
// Created by Artem on 07.02.2024.
//

#include "map/tiles_storage.h"
#include "util/android_log.h"
#include <fstream>

bool USE_MEM_CACHE = true;

Tile* TilesStorage::getTile(int zoom, int x, int y, GetTileRequest* getTileRequest) {
    std::string key = Tile::makeKey(zoom, x, y);
    auto it = cacheTiles.find(key);
    if(it == cacheTiles.end() || !USE_MEM_CACHE) {
        Tile* newTile = getTileRequest->request(x, y, zoom);
        cacheTiles.insert({key, newTile});
        return newTile;
    }

    return it->second;
}

TilesStorage::TilesStorage(Cache* cache, GetTileRequest* getTileRequest)
    : cache(cache), request(getTileRequest) { }

TilesStorage::~TilesStorage() {
    delete request;
}

bool TilesStorage::existInMemory(int zoom, int x, int y) {
    std::string key = Tile::makeKey(zoom, x, y);
    auto it = cacheTiles.find(key);
    return it != cacheTiles.end();
}
