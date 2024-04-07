
#include <vector>
#include <geometry.hpp>
#include <array>
#include "tile.h"

#ifndef TUSA_POLYGON_HANDLER
#define TUSA_POLYGON_HANDLER

class PolygonHandler {
public:
    std::vector<PolygonPoint> points;
    std::vector<std::vector<PolygonPoint>> polygon;
    std::vector<std::vector<std::vector<PolygonPoint>>> polygons;
    std::vector<PolygonPoint> featurePoints = {};
    int pointsCount = 0;

    void ring_begin(uint32_t count) {
        points.clear();
        points.reserve(count);
    }

    void ring_point(vtzero::point point) {
        points.push_back({(PolygonCoord)point.x, (PolygonCoord)point.y});
        featurePoints.push_back({(PolygonCoord)point.x, (PolygonCoord)point.y});
        pointsCount++;
    }

    void ring_end(vtzero::ring_type type) {
        if(type == vtzero::ring_type::outer && polygon.size() > 0) {
            polygons.push_back(polygon);
            polygon.clear();
        }

        polygon.push_back(points);
    }
};

#endif