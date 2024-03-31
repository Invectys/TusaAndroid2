//
// Created by Artem on 27.03.2024.
//

#include <vector>
#include "style/style_expecter.h"
#include "util/android_log.h"

void StyleExpecter::registerLayer(std::vector<std::string> expectedLayerName,
                                  CSSColorParser::Color color) {
    if(styleRegistered)
        return;

    bool classCondition = className == expectedClassName || isFallbackByClassName;

    bool layerNameCondition = std::find(expectedLayerName.begin(), expectedLayerName.end(), layerName) != expectedLayerName.end();
    if(layerNameCondition && classCondition) {
        colors[currentIndex] = color;
        selectedIndex = currentIndex;
        styleRegistered = true;
    }
    currentIndex++;

    isFallbackByClassName = false;
    expectedClassName = "";
}

StyleExpecter::StyleExpecter(
        layer_map_type props,
        std::string layerName,
        CSSColorParser::Color (&colors)[Style::maxGeometryHeaps]
) : layerName(layerName), colors(colors) {
    for(auto prop : props) {
        if(prop.first == "class") {
            className = boost::get<std::string>(prop.second);
        }
    }
}




