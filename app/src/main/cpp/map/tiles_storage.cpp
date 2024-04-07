//
// Created by Artem on 07.02.2024.
//

#include "map/tiles_storage.h"
#include "util/android_log.h"
#include <fstream>

bool USE_MEM_CACHE = true;

Tile* TilesStorage::getTile(int zoom, int x, int y) {
    std::string key = Tile::makeKey(zoom, x, y);
    auto it = cacheTiles.find(key);
    if(it == cacheTiles.end() || !USE_MEM_CACHE) {
        Tile* newTile = request->loadVectorTile(zoom, x, y);
        cacheTiles.insert({key, newTile});
        return newTile;
    }

    return it->second;
}

TilesStorage::TilesStorage(Cache* cache)
    : cache(cache) { }

TilesStorage::~TilesStorage() {
    delete request;
}
