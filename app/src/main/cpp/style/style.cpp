//
// Created by Artem on 09.02.2024.
//

#include "style/style.h"
#include <fstream>
#include <string>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include "util/android_log.h"
#include "style/style_expecter.h"
#include <vector>

struct StringExtractor : public boost::static_visitor<std::string> {
    std::string operator()(const std::string& str) const {
        return str;
    }

    template<typename T>
    std::string operator()(const T& /*unused*/) const {
        return ""; // Or throw an exception if necessary
    }
};

class TypeVisitor : public boost::static_visitor<std::string> {
public:
    template <typename T>
    std::string operator()(const T&) const {
        return typeid(T).name();
    }
};

short Style::determineStyle(
    std::string layerName,
    layer_map_type props
) {
    std::vector<std::string> ignoreLayers;
    ignoreLayers.push_back("hillshade");
    ignoreLayers.push_back("natural_label");
    ignoreLayers.push_back("place_label");
    if(std::find(ignoreLayers.begin(), ignoreLayers.end(), layerName) != ignoreLayers.end()) {
        return -1;
    }


    StyleExpecter styleExpecter = StyleExpecter(
            props,
            layerName,
            colorsOfHeaps,
            lineWidth
    );

    // Рендрится последним
    styleExpecter.registerLayer({"place_label"}, CSSColorParser::parse("rgb(169, 255, 2)"));

    styleExpecter.registerLayer({"airport_label"}, CSSColorParser::parse("rgb(255, 0, 0)"));

    uint64_t adminLevel = 0;
    if(layerName == "admin") {
        auto adminIso = props["iso_3166_1"];
        auto adminLevelVariant = props["admin_level"];
        std::string iso3166Admin = boost::apply_visitor(StringExtractor(), adminIso);
        adminLevel = boost::get<uint64_t>(adminLevelVariant);
    }

    if(adminLevel == 0) {
        styleExpecter.useLineWidth = 10;
        styleExpecter.registerLayer({"admin"}, CSSColorParser::parse("rgb(94, 0, 172)"));
    } else styleExpecter.currentIndex++;

    if(adminLevel == 1) {
        styleExpecter.useLineWidth = 2;
        styleExpecter.registerLayer({"admin"}, CSSColorParser::parse("rgba(94, 0, 172, 0.7)"));
    } else styleExpecter.currentIndex++;

    if(adminLevel >= 2) {
        styleExpecter.useLineWidth = 0.2;
        styleExpecter.registerLayer({"admin"}, CSSColorParser::parse("rgba(94, 0, 172, 0.5)"));
    } else styleExpecter.currentIndex++;


    styleExpecter.expectedClassName = "motorway";
    styleExpecter.useLineWidth = 20;
    styleExpecter.registerLayer({"road"}, CSSColorParser::parse("rgb(254, 229, 154)"));

    styleExpecter.expectedClassName = "trunk";
    styleExpecter.useLineWidth = 20;
    styleExpecter.registerLayer({"road"}, CSSColorParser::parse("rgb(254, 229, 154)"));

    styleExpecter.expectedClassName = "primary";
    styleExpecter.useLineWidth = 10;
    styleExpecter.registerLayer({"road"}, CSSColorParser::parse("rgb(233, 233, 237)"));

//    styleExpecter.expectedClassName = "motorway_link";
//    styleExpecter.useLineWidth = 20;
//    styleExpecter.registerLayer({"road"}, CSSColorParser::parse("rgb(254, 229, 154)"));

//
//    styleExpecter.expectedClassName = "primary_link";
//    styleExpecter.useLineWidth = 20;
//    styleExpecter.registerLayer({"road"}, CSSColorParser::parse("rgb(254, 229, 154)"));

//    styleExpecter.expectedClassName = "secondary";
//    styleExpecter.useLineWidth = 15;
//    styleExpecter.registerLayer({"road"}, CSSColorParser::parse("rgb(233, 233, 237)"));
//
    styleExpecter.useLineWidth = 5;
    styleExpecter.isFallbackByClassName = true;
    styleExpecter.registerLayer({"road"}, CSSColorParser::parse("rgb(233, 233, 237)"));


    styleExpecter.registerLayer({"water"}, CSSColorParser::parse("rgb(0, 186, 255)"));

    styleExpecter.isFallbackByClassName = true;
    styleExpecter.registerLayer({"waterway", "aeroway"}, CSSColorParser::parse("rgb(0, 227, 255)"));


    styleExpecter.registerLayer({"building"}, CSSColorParser::parse("rgb(237, 237, 237)"));


    styleExpecter.expectedClassName = "sand";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(236, 251, 228)"));

    styleExpecter.expectedClassName = "park";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(192, 240, 168)"));

    styleExpecter.expectedClassName = "scrub";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(224, 246, 213)"));

    styleExpecter.expectedClassName = "agriculture";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(243, 252, 238)"));

    styleExpecter.expectedClassName = "airport";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(244, 243, 251)"));

    styleExpecter.expectedClassName = "glacier";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(204, 233, 245)"));

    styleExpecter.expectedClassName = "cemetery";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(210, 236, 198)"));

    styleExpecter.expectedClassName = "grass";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(243, 252, 238)"));

    styleExpecter.expectedClassName = "wood";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(194, 239, 179)"));

    styleExpecter.expectedClassName = "hospital";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(246, 238, 244)"));

    styleExpecter.expectedClassName = "pitch";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(217, 248, 186)"));

    styleExpecter.expectedClassName = "school";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(241, 236, 228)"));

    styleExpecter.expectedClassName = "commercial_area";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(252, 244, 232)"));

    styleExpecter.expectedClassName = "parking";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(245, 245, 245)"));

    styleExpecter.expectedClassName = "residential";
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(255, 255, 255)"));

    styleExpecter.isFallbackByClassName = true;
    styleExpecter.registerLayer({"landuse"}, CSSColorParser::parse("rgb(242, 242, 242)"));


    styleExpecter.expectedClassName = "national_park";
    styleExpecter.registerLayer({"landuse_overlay"}, CSSColorParser::parse("rgb(192, 220, 178)"));

    styleExpecter.expectedClassName = "wetland";
    styleExpecter.registerLayer({"landuse_overlay"}, CSSColorParser::parse("rgb(50, 168, 141)"));


    styleExpecter.expectedClassName = "wood";
    styleExpecter.registerLayer({"landcover"}, CSSColorParser::parse("rgba(54, 199, 51, 0.8)"));

    styleExpecter.expectedClassName = "snow";
    styleExpecter.registerLayer({"landcover"}, CSSColorParser::parse("rgb(204, 233, 245)"));

    styleExpecter.expectedClassName = "scrub";
    styleExpecter.registerLayer({"landcover"}, CSSColorParser::parse("rgba(142, 231, 101, 0.8)"));

    styleExpecter.isFallbackByClassName = true;
    styleExpecter.registerLayer({"landcover"}, CSSColorParser::parse("rgba(204, 244, 185, 1.0)"));


    if(styleExpecter.selectedIndex == -1) {
        LOGI("Don't draw layer %s. Class = %s", layerName.data(), styleExpecter.getClassName().data());
    }
    return styleExpecter.selectedIndex;
}

Style::Style() {
    colorsOfHeaps[0] = CSSColorParser::parse("rgb(255, 0, 0)");
}

CSSColorParser::Color Style::getColorOfGeometryHeap(short style) {
    return colorsOfHeaps[style];
}



