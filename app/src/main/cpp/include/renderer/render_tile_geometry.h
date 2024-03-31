//
// Created by Artem on 30.03.2024.
//

#ifndef TUSA_ANDROID_RENDER_TILE_GEOMETRY_H
#define TUSA_ANDROID_RENDER_TILE_GEOMETRY_H


#include "map/tile.h"
#include "util/matrices.h"
#include "shader/shaders_bucket.h"

class RenderTileGeometry {
public:
    RenderTileGeometry(std::shared_ptr<ShadersBucket>);
    void render(Matrix4 pvm, Matrix4 modelMatrix, Tile *tile);
private:
    std::shared_ptr<ShadersBucket> shadersBucket;
};


#endif //TUSA_ANDROID_RENDER_TILE_GEOMETRY_H
