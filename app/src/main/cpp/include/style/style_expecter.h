//
// Created by Artem on 27.03.2024.
//

#ifndef TUSA_ANDROID_STYLE_EXPECTER_H
#define TUSA_ANDROID_STYLE_EXPECTER_H


#include "csscolorparser/csscolorparser.h"
#include "style.h"

class StyleExpecter {
public:
    StyleExpecter(
            layer_map_type props,
            std::string layerName,
            CSSColorParser::Color (&colors)[Style::maxGeometryHeaps],
            float (&lineWidth)[Style::maxGeometryHeaps]
    );

    short currentIndex = 1;
    short selectedIndex = -1;
    bool isFallbackByClassName = false;
    std::string expectedClassName = "";
    float useLineWidth = 0;

    void registerLayer(std::vector<std::string> expectedLayerName, CSSColorParser::Color color);
    std::string getClassName() { return className; }
private:
    CSSColorParser::Color (&colors)[Style::maxGeometryHeaps];
    float (&lineWidth)[Style::maxGeometryHeaps];
    bool styleRegistered = false;
    std::string layerName;
    std::string className;
};


#endif //TUSA_ANDROID_STYLE_EXPECTER_H
