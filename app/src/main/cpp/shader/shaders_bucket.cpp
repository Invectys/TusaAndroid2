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

void ShadersBucket::loadShaders(AAssetManager* assetManager) {
    long plainVertexSize;
    std::string plain_vertex = loadShader("shaders/plain.vert", assetManager, plainVertexSize);
    shadersCode.insert({"plain.vert", {plain_vertex, plainVertexSize}});

    long plainFragmentSize;
    std::string plain_fragment = loadShader("shaders/plain.frag", assetManager, plainFragmentSize);
    shadersCode.insert({"plain.frag", {plain_fragment, plainVertexSize}});
}

const char *ShadersBucket::loadShader(const char *filename, AAssetManager* assetManager, long& size) {
    AAsset* asset = AAssetManager_open(assetManager, filename, AASSET_MODE_BUFFER);
    size = AAsset_getLength(asset);
    char *buffer = (char*) malloc(sizeof(char)*size);
    AAsset_read(asset, buffer, size);
    AAsset_close(asset);
    return buffer;
}

ShadersBucket::ShadersBucket() {}

std::shared_ptr<PlainShader> ShadersBucket::createPlainShader(AAssetManager* assetManager) {
    return std::shared_ptr<PlainShader>(new PlainShader(
            assetManager, "shaders/plain.vert", "shaders/plain.frag"
    ));
}

void ShadersBucket::compileAllShaders(AAssetManager* assetManager) {
    plainShader = createPlainShader(assetManager);
    symbolShader = std::shared_ptr<SymbolShader>(new SymbolShader(
            assetManager, "shaders/symbol.vert", "shaders/symbol.frag"
    ));
}

ShadersBucket::~ShadersBucket() {

}






