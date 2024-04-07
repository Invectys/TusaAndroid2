//
// Created by Artem on 30.03.2024.
//

#include "symbols/symbols.h"
#include "GLES2/gl2.h"
#include "util/matrices.h"

void Symbols::loadFont(AAssetManager *assetManager) {
    if (FT_Init_FreeType(&ft)) {
        return;
    }

    AAsset* fontFile = AAssetManager_open(assetManager, "fonts/Arial Black.ttf", AASSET_MODE_BUFFER);
    off_t fontDataSize = AAsset_getLength(fontFile);
    FT_Byte *fontData = new FT_Byte[fontDataSize];
    AAsset_read(fontFile, fontData, (size_t) fontDataSize);
    AAsset_close(fontFile);

    if (FT_New_Memory_Face(ft, (const FT_Byte*)fontData, (FT_Long)fontDataSize, 0, &face)){
        return;
    }

    FT_Set_Pixel_Sizes(face, 0, 300);
}

void Symbols::createFontTextures() {
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    if(loadOnlySelectedCharCodes) {
        for(auto& selectedChar : selectedCharCodesForLoading) {
            prepareCharForRendering(selectedChar);
        }
        return;
    }

    for(unsigned short charcode = 0; charcode <= 255; charcode++) {
        prepareCharForRendering(charcode);
    }
}

Symbol Symbols::getSymbol(char c) {
    return symbols[c];
}

void Symbols::prepareCharForRendering(unsigned short charcode) {
    if(FT_Load_Char(face, charcode, FT_LOAD_RENDER)) {
        return;
    }

    unsigned int texture;
    glGenTextures(1, &texture);

    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_ALPHA,
            face->glyph->bitmap.width,
            face->glyph->bitmap.rows,
            0,
            GL_ALPHA,
            GL_UNSIGNED_BYTE,
            face->glyph->bitmap.buffer
    );

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    Symbol symbol = Symbol(
            texture,
            face->glyph->bitmap.width,
            face->glyph->bitmap.rows,
            face->glyph->bitmap_left,
            face->glyph->bitmap_top,
            face->glyph->advance.x
    );

    symbols.insert(std::pair<char, Symbol>(charcode, symbol));
}

void Symbols::renderText(std::string text, float x, float y, Matrix4 translate, float symbolScale) {
    std::vector<std::tuple<Symbol, float, float, float>> forRender {};

    float textWidth = 0;
    float sumHeight = 0;
    std::string::const_iterator iterator;
    for(iterator = text.begin(); iterator != text.end(); iterator++) {
        auto symbol = getSymbol(*iterator);

        float w = symbol.width * symbolScale;
        float h = symbol.rows * symbolScale;

        float xPixelsShift = (symbol.advance >> 6) * symbolScale;
        textWidth += xPixelsShift;
        sumHeight += h;

        forRender.push_back({symbol, w, h, xPixelsShift});
    }

    float averageHeight = sumHeight / text.length();
    x = x - textWidth / 2;
    y -= averageHeight / 2;

    auto symbolShader = shadersBucket->symbolShader;
    glUseProgram(symbolShader->program);

    for(auto& charRender : forRender) {
        Symbol symbol = std::get<0>(charRender);
        unsigned int textureId = symbol.textureId;
        float w = std::get<1>(charRender);
        float h = std::get<2>(charRender);
        float pixelsShift = std::get<3>(charRender);

        float xPos = x + symbol.bitmapLeft * symbolScale;
        float yPos = y - (symbol.rows - symbol.bitmapTop ) * symbolScale;

        float points[] = {
                xPos, (yPos + h),
                xPos, yPos,
                xPos + w, yPos,
                xPos + w, (yPos + h)
        };

        GLfloat textureCords[] = {
                0.0f, 0.0f,
                0.0f, 1.0f,
                1.0f, 1.0f,
                1.0f, 0.0f
        };

        unsigned int indices[6] = {
                2, 3, 0,
                0, 1, 2
        };

        glUniformMatrix4fv(symbolShader->getMatrixLocation(), 1, GL_FALSE, translate.get());
        glBindTexture(GL_TEXTURE_2D, textureId);

        glUniform1i(symbolShader->getTextureLocation(), 0);

        glVertexAttribPointer(symbolShader->getTextureCord(), 2, GL_FLOAT, GL_FALSE, 0, textureCords);
        glEnableVertexAttribArray(symbolShader->getTextureCord());

        glVertexAttribPointer(symbolShader->getPosLocation(), 2, GL_FLOAT, GL_FALSE, 0, points);
        glEnableVertexAttribArray(symbolShader->getPosLocation());
        glUniform4f(symbolShader->getColorLocation(), 1.0, 0.0, 0.0f, 1.0f);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, indices);

        x += pixelsShift;
    }
}

Symbols::Symbols(std::shared_ptr<ShadersBucket> shadersBucket): shadersBucket(shadersBucket){

}
