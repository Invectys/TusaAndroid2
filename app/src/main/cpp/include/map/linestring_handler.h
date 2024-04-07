
#include <vector>
#include <geometry.hpp>

#ifndef TUSA_LINESTRING_HANDLER
#define TUSA_LINESTRING_HANDLER

class LineStringHandler {
public:
    std::vector<std::vector<vtzero::point>> lines = {};
    std::vector<vtzero::point> points;
    size_t pointsCount = 0;

    void linestring_point(vtzero::point point) noexcept {
        points.push_back(point);
        pointsCount++;
    }

    void linestring_end() {
        lines.push_back(points);
        points.clear();
    }

    void linestring_begin(uint32_t count) {
        points.reserve(count);
    }
};


#endif