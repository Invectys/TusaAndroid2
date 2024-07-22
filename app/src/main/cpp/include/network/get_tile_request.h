//
// Created by Artem on 20.01.2024.
//

#ifndef TUSA_REQUEST_H
#define TUSA_REQUEST_H


#include <jni.h>
#include <thread>
#include "cache/cache.h"
#include "map/tile.h"
#include "style/style.h"

class GetTileRequest {
public:
    GetTileRequest(Cache* cache, JNIEnv *env);
    ~GetTileRequest() {
        mainThreadEnv->DeleteGlobalRef(requestTileObjectGlobal);
        mainThreadEnv->DeleteGlobalRef(requestTileClassGlobal);
    }

    Tile* loadVectorTile(int zoom, int x, int y);
    Tile* request(int x, int y, int z);

    void setEnv(JNIEnv* jniEnv) {
        parallelThreadEnvironments[std::this_thread::get_id()] = jniEnv;
    }
private:
    Cache* cache;
    JNIEnv *mainThreadEnv;
    std::map<std::thread::id, JNIEnv*> parallelThreadEnvironments;

    jclass requestTileClassGlobal;
    jobject requestTileObjectGlobal;
};


#endif //TUSA_REQUEST_H
