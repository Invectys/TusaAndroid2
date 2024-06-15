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
#include "gl/open_gl_interface.h"

class Map : public IOpenGl{
public:
    Map(Cache* cache);

    void render() override;
    void onSurfaceChanged(int w, int h) override;
    void onSurfaceCreated(AAssetManager* assetManager) override;
    void noOpenGlContextInit(AAssetManager* assetManager, float scaleFactor) override;
    void drag(float dx, float dy) override;
    void scale(float scaleFactor) override;
    void doubleTap() override;

private:

    float zoom = 0;
    Cache* cache;
    std::shared_ptr<ShadersBucket> shadersBucket = std::shared_ptr<ShadersBucket>(new ShadersBucket());
    Renderer renderer = Renderer(shadersBucket, cache);

};


#endif //TUSA_MAP_H
