//
// Created by Artem on 01.01.2024.
//

#include "map/map.h"
#include "renderer/renderer.h"
#include "network/request.h"
#include "util/frustrums.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

void Map::onSurfaceCreated(AAssetManager *assetManager) {
    shadersBucket->compileAllShaders(assetManager);
    renderer.getSymbols()->createFontTextures();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_STENCIL_TEST);


//    AAsset *asset = AAssetManager_open(assetManager, "images/helmet-32.png", AASSET_MODE_BUFFER);
//    off_t bufferLength = AAsset_getLength(asset);
//    char* buffer = (char*)malloc(bufferLength);
//    AAsset_read(asset, buffer, bufferLength);
//    AAsset_close(asset);
//
//    int width, height, channels;
//    stbi_uc* image = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(buffer), bufferLength, &width, &height, &channels, STBI_rgb_alpha);
//
//    unsigned int texture;
//    glGenTextures(1, &texture);
//    glBindTexture(GL_TEXTURE_2D, texture);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
//
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//
//    free(buffer);
}

void Map::onSurfaceChanged(int w, int h) {
    renderer.onSurfaceChanged(w, h);

    glViewport(0, 0, w, h);
}

void Map::render() {
    renderer.renderFrame();
}

void Map::noOpenGlContextInit(AAssetManager *assetManager, float scaleFactor) {
    shadersBucket->loadShaders(assetManager);
    renderer.setupNoOpenGLMapState(scaleFactor, assetManager);
}

Map::Map(Cache *cache): cache(cache) {}

void Map::drag(float dx, float dy) {
    renderer.drag(dx, dy);
}

void Map::scale(float scaleFactor) {
    renderer.scale(scaleFactor);
}

void Map::doubleTap() {
    renderer.doubleTap();
}
