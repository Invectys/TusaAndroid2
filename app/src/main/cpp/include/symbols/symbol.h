//
// Created by Artem on 30.03.2024.
//

#ifndef TUSA_ANDROID_SYMBOL_H
#define TUSA_ANDROID_SYMBOL_H

#include "ft2build.h"
#include FT_FREETYPE_H

class Symbol {
public:
    Symbol();
    Symbol(
        unsigned int textureId,
        unsigned int width,
        unsigned int rows,
        FT_Int bitmapLeft,
        FT_Int bitmapTop,
        FT_Pos advance
    );

    unsigned int textureId;
    unsigned int width;
    unsigned int rows;
    FT_Int bitmapLeft;
    FT_Int bitmapTop;
    FT_Pos advance;
};


#endif //TUSA_ANDROID_SYMBOL_H
