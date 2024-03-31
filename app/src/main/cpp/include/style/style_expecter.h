//
// Created by Artem on 27.03.2024.
//

#ifndef TUSA_ANDROID_STYLE_EXPECTER_H
#define TUSA_ANDROID_STYLE_EXPECTER_H


#include "csscolorparser/csscolorparser.h"
#include "style.h"

class StyleExpecter {
public:
    StyleExpecter(layer_map_type props, std::string layerName, CSSColorParser::Color (&colors)[Style::maxGeometryHeaps]);

    short currentIndex = 1;
    short selectedIndex = -1;
    CSSColorParser::Color (&colors)[Style::maxGeometryHeaps];

    bool isFallbackByClassName = false;
    std::string expectedClassName = "";

    void registerLayer(std::vector<std::string> expectedLayerName, CSSColorParser::Color color);
private:
    bool styleRegistered = false;
    std::string layerName;
    std::string className;
};


#endif //TUSA_ANDROID_STYLE_EXPECTER_H
