//
// Created by Artem on 01.01.2024.
//

#include "map/map.h"
#include "renderer/renderer.h"
#include "network/get_tile_request.h"
#include "util/frustrums.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

void Map::onSurfaceCreated(AAssetManager *assetManager) {
    shadersBucket->compileAllShaders(assetManager);

    renderer.onSurfaceCreated(assetManager);
}

void Map::onSurfaceChanged(int w, int h) {
    renderer.onSurfaceChanged(w, h);
}

void Map::render() {
    renderer.renderFrame();
}

void Map::noOpenGlContextInit(AAssetManager *assetManager, float scaleFactor, JNIEnv *env) {
    renderer.setupNoOpenGLMapState(scaleFactor, assetManager, env);
}

Map::Map(Cache *cache): cache(cache) { }

void Map::drag(float dx, float dy) {
    renderer.drag(dx, dy);
}

void Map::scale(float scaleFactor) {
    renderer.scale(scaleFactor);
}

void Map::doubleTap() {
    renderer.doubleTap();
}

void Map::onStop() {
    renderer.onStop();
}

