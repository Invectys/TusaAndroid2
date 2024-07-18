//
// Created by Artem on 30.03.2024.
//

#ifndef TUSA_ANDROID_SYMBOLS_H
#define TUSA_ANDROID_SYMBOLS_H

#include <android/asset_manager.h>
#include <map>
#include <string>
#include <vector>
#include "symbol.h"
#include "ft2build.h"
#include FT_FREETYPE_H
#include "util/matrices.h"
#include "shader/shaders_bucket.h"

class Symbols {
public:
    Symbols(std::shared_ptr<ShadersBucket> shadersBucket);

    void loadFont(AAssetManager *assetManager);
    void createFontTextures();
    Symbol getSymbol(char c);

    void renderText(std::string text, float x, float y, Matrix4 translate, float scale);
    void renderText2(std::string text, Matrix4 m, float x, float y, float z, float symbolScale);


    ~Symbols() {
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
    }

private:
    void prepareCharForRendering(unsigned short charcode);

    bool loadOnlySelectedCharCodes = false;
    std::vector<unsigned short> selectedCharCodesForLoading = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

    std::shared_ptr<ShadersBucket> shadersBucket;
    FT_Face face;
    FT_Library ft;
    std::map<char, Symbol> symbols = {};
};


#endif //TUSA_ANDROID_SYMBOLS_H
