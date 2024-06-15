//
// Created by Artem on 20.12.2023.
//

#include "shader/shaders_bucket.h"
#include "shader/shaders_bucket.h"
#include <iostream>
#include <fstream>
#include <string>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>


ShadersBucket::ShadersBucket() {}

void ShadersBucket::compileAllShaders(AAssetManager* assetManager) {
    plainShader = std::shared_ptr<PlainShader>(new PlainShader(
            assetManager, "shaders/plain.vert", "shaders/plain.frag"
    ));
    symbolShader = std::shared_ptr<SymbolShader>(new SymbolShader(
            assetManager, "shaders/symbol.vert", "shaders/symbol.frag"
    ));
    planetShader = std::shared_ptr<PlanetShader>(new PlanetShader(
            assetManager, "shaders/planet.vert", "shaders/planet.frag"
    ));
}

ShadersBucket::~ShadersBucket() {

}






