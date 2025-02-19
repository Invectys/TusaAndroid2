//
// Created by Artem on 20.12.2023.
//

#ifndef TUSA_SHADERS_BUCKET_H
#define TUSA_SHADERS_BUCKET_H

#include <map>
#include <android/asset_manager.h>
#include "plain_shader.h"
#include "symbol_shader.h"
#include "shaders_bucket.h"
#include "planet_shader.h"
#include <boost/shared_ptr.hpp>

class ShadersBucket {
public:
    ShadersBucket();
    ~ShadersBucket();

    void compileAllShaders(AAssetManager* assetManager);

    std::shared_ptr<PlainShader> plainShader;
    std::shared_ptr<SymbolShader> symbolShader;
    std::shared_ptr<PlanetShader> planetShader;
};


#endif //TUSA_SHADERS_BUCKET_H
