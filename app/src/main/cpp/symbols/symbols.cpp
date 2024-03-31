//
// Created by Artem on 30.03.2024.
//

#include "symbols/symbols.h"
#include "GLES2/gl2.h"

void Symbols::loadFont(AAssetManager *assetManager) {
    if (FT_Init_FreeType(&ft)) {
        return;
    }

    AAsset* fontFile = AAssetManager_open(assetManager, "fonts/nyashasans.ttf", AASSET_MODE_BUFFER);
    off_t fontDataSize = AAsset_getLength(fontFile);
    FT_Byte *fontData = new FT_Byte[fontDataSize];
    AAsset_read(fontFile, fontData, (size_t) fontDataSize);
    AAsset_close(fontFile);

    if (FT_New_Memory_Face(ft, (const FT_Byte*)fontData, (FT_Long)fontDataSize, 0, &face)){
        return;
    }

    FT_Set_Pixel_Sizes(face, 0, 48);
    FT_Load_Char(face, 'X', FT_LOAD_RENDER);
}

void Symbols::createFontTextures() {
    char symbolCode = 'a';
    if(FT_Load_Char(face, symbolCode, FT_LOAD_RENDER)) {
        return;
    }

    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB,
        face->glyph->bitmap.width,
        face->glyph->bitmap.rows,
        0,
        1,
        GL_BYTE,
        face->glyph->bitmap.buffer
    );

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    Symbol symbol = {
            texture,
            face->glyph->bitmap.width,
            face->glyph->bitmap.rows,
            face->glyph->bitmap_left,
            face->glyph->bitmap_top,
            face->glyph->advance.x
    };
    symbols.insert(std::pair<char, Symbol>(symbolCode, symbol));
}

Symbol Symbols::getSymbol(char c) {
    return symbols[c];
}
