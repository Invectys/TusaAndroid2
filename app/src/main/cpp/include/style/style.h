//
// Created by Artem on 09.02.2024.
//

#ifndef TUSA_STYLE_H
#define TUSA_STYLE_H

#include <android/asset_manager.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <map>
#include <array>
#include <GLES2/gl2.h>
#include "csscolorparser/csscolorparser.h"
#include <boost/variant.hpp>

using layer_key_type = std::string; // must be something that can be converted from data_view
using layer_value_type = boost::variant<std::string, float, double, int64_t, uint64_t, bool>;
using layer_map_type = std::map<layer_key_type, layer_value_type>;

using namespace rapidjson;

class Style {
public:
    Style();
    static const short maxGeometryHeaps = 100;
    CSSColorParser::Color getColorOfGeometryHeap(short style);
    short determineStyle(std::string layerName, layer_map_type props);
    float getLineWidthOfHeap(short style) {
        if(lineWidth[style] != 0) {
            return lineWidth[style];
        }
        return defaultLineWidth;
    }
private:
    CSSColorParser::Color colorsOfHeaps[Style::maxGeometryHeaps] = {};
    float defaultLineWidth = 1;
    float lineWidth[Style::maxGeometryHeaps] = {};
};


#endif //TUSA_STYLE_H
