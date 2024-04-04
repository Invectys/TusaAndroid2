//
// Created by Artem on 30.03.2024.
//

#include "symbols/symbol.h"

Symbol::Symbol(unsigned int textureId,
               unsigned int width,
               unsigned int rows,
               FT_Int bitmapLeft,
               FT_Int bitmapTop,
               FT_Pos advance) :
               textureId(textureId),
               width(width),
               rows(rows),
               bitmapLeft(bitmapLeft),
               bitmapTop(bitmapTop),
               advance(advance) { }

Symbol::Symbol() {

}
