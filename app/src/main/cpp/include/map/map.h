//
// Created by Artem on 01.01.2024.
//

#ifndef TUSA_MAP_H
#define TUSA_MAP_H

#include "../shader/shaders_bucket.h"
#include "renderer/renderer.h"
#include "network/request.h"
#include "tiles_storage.h"
#include "style/style.h"
#include "mutex"

class Map {
public:
    Map(Cache* cache);

    void noOpenGlContextInit(AAssetManager* assetManager, float scaleFactor);
    void onSurfaceCreated(AAssetManager* assetManager);
    void onSurfaceChanged(int w, int h);
    void drag(float dx, float dy);
    void scale(float scaleFactor);
    void doubleTap();
    void render();
private:

    float zoom = 0;
    Cache* cache;
    std::shared_ptr<ShadersBucket> shadersBucket = std::shared_ptr<ShadersBucket>(new ShadersBucket());
    Renderer renderer = Renderer(shadersBucket, cache);

};


#endif //TUSA_MAP_H
