//
// Created by Artem on 28.03.2024.
//

#ifndef TUSA_ANDROID_MAP_DRAW_CALL_H
#define TUSA_ANDROID_MAP_DRAW_CALL_H

#include <vector>
#include <GLES2/gl2.h>
#include "csscolorparser/csscolorparser.h"

class MapDrawCall {
public:
    MapDrawCall();

    int openGLDrawType;
    std::vector<int> coords;
    std::vector<unsigned int> indices;

    CSSColorParser::Color color;
};


#endif //TUSA_ANDROID_MAP_DRAW_CALL_H
