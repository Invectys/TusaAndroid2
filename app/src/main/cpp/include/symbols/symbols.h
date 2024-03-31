//
// Created by Artem on 30.03.2024.
//

#ifndef TUSA_ANDROID_SYMBOLS_H
#define TUSA_ANDROID_SYMBOLS_H

#include "ft2build.h"
#include FT_FREETYPE_H
#include <android/asset_manager.h>
#include <map>
#include "symbol.h"

class Symbols {
public:
    void loadFont(AAssetManager *assetManager);
    void createFontTextures();
    Symbol getSymbol(char c);


    ~Symbols() {
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
    }
private:
    FT_Face face;
    FT_Library ft;
    std::map<char, Symbol> symbols = {};
};


#endif //TUSA_ANDROID_SYMBOLS_H
