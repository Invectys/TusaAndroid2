//
// Created by Artem on 08.02.2024.
//

//#include <mapbox/vector_tile.hpp>
#include "map/tile.h"
#include "util/android_log.h"
#include "util/feature_geometry_block.h"
#include "map/linestring_handler.h"
#include "boost/variant.hpp"
#include "map/polygon_handler.h"
#include <sstream>
#include <chrono>
#include <mapbox/earcut.hpp>
#include <array>
#include <vector>
#include <vector_tile.hpp>
#include <chrono>
#include <thread>


bool DEBUG_TILE_PARSE = false;

double shoelace(std::vector<PolygonPoint> points) {
    double leftSum = 0.0;
    double rightSum = 0.0;

    for (int i = 0; i < points.size(); ++i) {
        int j = (i + 1) % points.size();
        leftSum  += points[i][0] * points[j][1];
        rightSum += points[j][0] * points[i][1];
    }

    return leftSum - rightSum;
}

Tile::Tile(int zoom, int x, int y, vtzero::vector_tile *tile)
: zoom(zoom), x(x), y(y) {

    auto start = std::chrono::high_resolution_clock ::now();
    parseMethod1(zoom, x, y, tile);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    LOGI("Tile parse time: %f", duration.count());

    delete tile;
}

std::string Tile::makeKey(int zoom, int x, int y) {
    return std::to_string(zoom) + std::to_string(x) + std::to_string(y);
}

void Tile::parseMethod1(int zoom, int x, int y, vtzero::vector_tile *tile) {

//    auto currentTime = std::chrono::system_clock::now();
//    auto wakeupTime = currentTime + std::chrono::seconds(5);
//    std::this_thread::sleep_until(wakeupTime);

    std::forward_list<FeatureGeometryBlock<Geometry<LineCoord, LineIndic>>> linesFeatureGeomBlockList[Style::maxGeometryHeaps];
    std::forward_list<std::pair<std::vector<std::vector<PolygonPoint>>, std::vector<PolygonIndic>>> polygonsFeatureGeomBlockList[Style::maxGeometryHeaps];

    size_t linesPointsCount[Style::maxGeometryHeaps] = {};
    size_t linesResultIndicesCount[Style::maxGeometryHeaps] = {};

    size_t polygonsPointsCount[Style::maxGeometryHeaps] = {};
    size_t polygonsIndicesCount[Style::maxGeometryHeaps] = {};

    std::fill(linesPointsCount, linesPointsCount + Style::maxGeometryHeaps, 0);
    std::fill(linesResultIndicesCount, linesResultIndicesCount + Style::maxGeometryHeaps, 0);

    std::fill(polygonsIndicesCount, polygonsIndicesCount + Style::maxGeometryHeaps, 0);
    std::fill(polygonsPointsCount, polygonsPointsCount + Style::maxGeometryHeaps, 0);

    while (auto layer = tile->next_layer()) {
        auto name = layer.name().to_string();
        std::size_t feature_count = layer.num_features();
        uint32_t extent = layer.extent();
        while (auto feature = layer.next_feature()) {
            auto geomType = feature.geometry_type();
            auto props = create_properties_map<layer_map_type>(feature);
            auto styleIndex = style.determineStyle(name, props);
            if(styleIndex == -1) {
                continue;
            }

            if(geomType == vtzero::GeomType::LINESTRING) {
                auto lineHandler = LineStringHandler();
                vtzero::decode_linestring_geometry(feature.geometry(), lineHandler);
                size_t geomSize = lineHandler.lines.size();
                Geometry<LineCoord, LineIndic> *featureGeometry = new Geometry<LineCoord, LineIndic>[geomSize];
                for (size_t geomIndex = 0; geomIndex < geomSize; ++geomIndex) {
                    const auto& point_array = lineHandler.lines[geomIndex];
                    const size_t pointsSize = point_array.size();
                    linesPointsCount[styleIndex] += pointsSize;
                    LineCoord* lines = new LineCoord[pointsSize * 2];
                    for(int pointIndex = 0; pointIndex < pointsSize; ++pointIndex) {
                        auto point = point_array[pointIndex];
                        lines[pointIndex * 2] = point.x;
                        lines[pointIndex * 2 + 1] = -1 * point.y;
                    }

                    short amountOfInserts = 0;
                    if(pointsSize > 2) {
                        amountOfInserts = pointsSize - 2;
                    }
                    size_t indicesAmount = pointsSize + amountOfInserts;
                    LineIndic *indices = new LineIndic[indicesAmount];
                    for(int indicesIndex = 0; indicesIndex < indicesAmount; indicesIndex += 2) {
                        indices[indicesIndex] = indicesIndex / 2;
                        indices[indicesIndex + 1] = indicesIndex / 2 + 1;
                    }
                    linesResultIndicesCount[styleIndex] += indicesAmount;
                    featureGeometry[geomIndex] = Geometry<LineCoord, LineIndic>(lines, pointsSize, indices, indicesAmount);
                }

                linesFeatureGeomBlockList[styleIndex].push_front(FeatureGeometryBlock<Geometry<LineCoord, LineIndic>>(featureGeometry, geomSize));
            } else if(geomType == vtzero::GeomType::POLYGON) {
                auto polygonHandler = PolygonHandler();
                vtzero::decode_polygon_geometry(feature.geometry(), polygonHandler);
                auto& polygons = polygonHandler.polygons;
                polygons.push_back(polygonHandler.polygon);
                polygonsPointsCount[styleIndex] += polygonHandler.pointsCount;
                for(std::vector<std::vector<PolygonPoint>>& polygon : polygons) {
                    std::vector<PolygonIndic> indices = mapbox::earcut<PolygonIndic>(polygon);
                    polygonsIndicesCount[styleIndex] += indices.size();
                    polygonsFeatureGeomBlockList[styleIndex].push_front({polygon, indices});
                }
            }
        }
    }

    for(short useStyleIndex = 0; useStyleIndex < Style::maxGeometryHeaps; ++useStyleIndex) {
        size_t indicesShift = 0;
        size_t pointsShift = 0;
        LineIndic *resultLinesIndices = new LineIndic[linesResultIndicesCount[useStyleIndex]];
        LineCoord *resultLinesPoints = new LineCoord[linesPointsCount[useStyleIndex] * 2];
        auto& listOfGeometries = linesFeatureGeomBlockList[useStyleIndex];
        for(auto& arrayOfGeometries : listOfGeometries) {
            for(int useGeometry = 0; useGeometry < arrayOfGeometries.size; ++useGeometry) {
                auto& geometry = arrayOfGeometries.data[useGeometry];
                for(int pointIndex = 0; pointIndex < geometry.pointsCount * 2; ++pointIndex) {
                    auto& coordinate = geometry.points[pointIndex];
                    resultLinesPoints[pointIndex + pointsShift * 2] = coordinate;
                }
                for(int indicesIndex = 0; indicesIndex < geometry.indicesCount; ++indicesIndex) {
                    LineIndic in = geometry.indices[indicesIndex] + pointsShift;
                    resultLinesIndices[indicesIndex + indicesShift] = in;
                }
                indicesShift += geometry.indicesCount;
                pointsShift += geometry.pointsCount;
            }
        }


        if(DEBUG_TILE_PARSE) {
            std::stringstream ss;
            ss << "End geometry: ";
            for(int i = 0; i < linesPointsCount[useStyleIndex] * 2; ++i) {
                ss << i / 2 << ":" << resultLinesPoints[i] << ", ";
            }
            ss << "End indices: ";
            for(int i = 0; i < linesResultIndicesCount[useStyleIndex]; ++i) {
                ss << resultLinesIndices[i] << " ";
            }
            std::string resultInfo = ss.str();
            LOGI("%s", resultInfo.data());
        }

        resultLines[useStyleIndex] = Geometry<LineCoord, LineIndic>(
                resultLinesPoints, linesPointsCount[useStyleIndex],
                resultLinesIndices, linesResultIndicesCount[useStyleIndex]
        );

        PolygonIndic currentPointIndex = 0;
        PolygonIndic currentIndicIndex = 0;
        PolygonIndic currentIndicShift = 0;
        PolygonCoord *resultPolygonPoints = new PolygonCoord[polygonsPointsCount[useStyleIndex] * 2];
        PolygonIndic *resultPolygonIndices = new PolygonIndic[polygonsIndicesCount[useStyleIndex]];
        for(auto& polygonPair : polygonsFeatureGeomBlockList[useStyleIndex]) {
            auto rings = polygonPair.first;
            PolygonIndic useShiftIndices = 0;
            for(auto ring : rings) {
                useShiftIndices += ring.size();
                for(auto& point : ring) {
                    resultPolygonPoints[currentPointIndex] = point[0];
                    resultPolygonPoints[currentPointIndex + 1] = -1 * point[1];
                    currentPointIndex += 2;
                }
            }

            std::vector<PolygonIndic> indices = polygonPair.second;
            for(auto indic : indices) {
                resultPolygonIndices[currentIndicIndex++] = indic + currentIndicShift;
            }
            currentIndicShift += useShiftIndices;
        }


        if(DEBUG_TILE_PARSE) {
            std::stringstream ss;
            ss << "Polygon End geometry: ";
            for(int i = 0; i < polygonsPointsCount[useStyleIndex] * 2; ++i) {
                ss << i / 2 << ":" << resultPolygonPoints[i] << ", ";
            }
            ss << "Polygon End indices: ";
            for(int i = 0; i < polygonsIndicesCount[useStyleIndex]; ++i) {
                ss << resultPolygonIndices[i] << " ";
            }
            std::string resultInfo = ss.str();
            LOGI("%s", resultInfo.data());
        }

        resultPolygons[useStyleIndex] = Geometry<PolygonCoord, PolygonIndic>(
                resultPolygonPoints, polygonsPointsCount[useStyleIndex],
                resultPolygonIndices, polygonsIndicesCount[useStyleIndex]
        );
    }
}
