//
// Created by Artem on 20.04.2024.
//

#ifndef TUSA_ANDROID_SPHERE_H
#define TUSA_ANDROID_SPHERE_H

#include <math.h>
#include <cmath>
#include <vector>
#include "map/mercator.h"
#include "util/android_log.h"

class Sphere {
public:
    float degLatitudeConstraint = 85.0511f;

    void generateTextureCoordinates() {
        for(size_t i = 0; i < sphere_vertices.size(); i++) {
            float coordinate = sphere_vertices[i];
        }
    }

    void generateSphereData3(int stackCount, int sectorCount, float radius,
                             float centerLatitudeRad,
                             float centerLongitudeRad,
                             float fromCenterDeltaRad
    ) {
        // clear memory of prev arrays
        std::vector<float>().swap(sphere_vertices);
        std::vector<unsigned int>().swap(sphere_indices);
        std::vector<float>().swap(normals);
        std::vector<float>().swap(texCords);
        std::vector<float>().swap(unitSquareCoordinates);

        float sectorStep = 2 * M_PI / sectorCount;
        float stackStep = M_PI / stackCount;

        int stackIndexIndicesStartFrom = -1;
        int stackIndexIndicesTo;

        int sectorIndexStartFrom = -1;
        int sectorIndexTo = 0;

        for(int stackIndex = 0; stackIndex <= stackCount; stackIndex++) {
            float stackAngle = -M_PI / 2.0 + stackStep * stackIndex;
            float latitude = stackAngle; //    - PI / 2 to PI / 2

            bool latitudeAngleValid = latitude >= centerLatitudeRad - fromCenterDeltaRad && latitude <= centerLatitudeRad + fromCenterDeltaRad;
            if (!latitudeAngleValid)
                continue;

//            if(!(stackIndex == 0 || stackIndex == 1 || stackIndex == 2))
//                continue;

            if(stackIndexIndicesStartFrom == -1) {
                stackIndexIndicesStartFrom = stackIndex;
            }
            stackIndexIndicesTo = stackIndex;

            // Это верхняя или нижняя точка
            bool isBottomPoint = stackIndex == 0;
            bool isUpPoint = stackIndex == stackCount;

            for(int sectorIndex = 0; sectorIndex <= sectorCount; sectorIndex++) {

                float sectorAngle = sectorStep * sectorIndex;
                float longitude = sectorAngle - M_PI;

                bool longitudeAngleValid = longitude >= centerLongitudeRad - fromCenterDeltaRad && longitude <= centerLongitudeRad + fromCenterDeltaRad;
                if(!longitudeAngleValid)
                    continue;

//                if(!(sectorIndex == 1 || sectorIndex == 2 || sectorIndex == 3 || sectorIndex == 4)) {
//                    continue;
//                }

                if(sectorIndexStartFrom == -1) {
                    sectorIndexStartFrom = sectorIndex;
                }
                if(sectorIndex > sectorIndexTo) {
                    sectorIndexTo = sectorIndex;
                }

                float y = radius * sinf(stackAngle);
                float x = radius * cosf(stackAngle) * cosf(sectorAngle);
                float z = radius * cosf(stackAngle) * sinf(sectorAngle);

                sphere_vertices.push_back(x);
                sphere_vertices.push_back(y);
                sphere_vertices.push_back(z);

                float t = std::min(std::max(0.0f, 1.0f - (float) sectorIndex / (float) sectorCount), 1.0f);
                float s = CommonUtils::latitudeRadToY(stackAngle);
                texCords.push_back(t); // longitude x
                texCords.push_back(s); // latitude y

                unitSquareCoordinates.push_back(t);
                unitSquareCoordinates.push_back(s);

                if(isUpPoint || isBottomPoint) {
                    break;
                }
            }
        }

        bool bottomPointExists = stackIndexIndicesStartFrom == 0;
        bool upPointExists = stackIndexIndicesTo == stackCount;

        int stacksLastIndic = stackIndexIndicesTo - stackIndexIndicesStartFrom;
        int fullRings = stacksLastIndic + 1 - upPointExists - bottomPointExists;
        int pointsBySector = sectorIndexTo - sectorIndexStartFrom + 1;
        int sectorsLastCount = sectorIndexTo - sectorIndexStartFrom;

        int upPoint = fullRings * (pointsBySector) - 1 + upPointExists + bottomPointExists;
        for(int stackIndex = 0; stackIndex <= stacksLastIndic; stackIndex++)
        {
            // верхняя шапка
            if(stackIndex == stacksLastIndic && upPointExists) {
                for(int sectorIndex = 0; sectorIndex < sectorsLastCount; sectorIndex++) {
                    sphere_indices.push_back(upPoint);
                    sphere_indices.push_back(upPoint - sectorIndex - 2);
                    sphere_indices.push_back(upPoint - sectorIndex - 1);
                }
                continue;
            }

            // нижняя шапка
            if(stackIndex == 0 && bottomPointExists) {
                for(int sectorIndex = 0; sectorIndex < sectorsLastCount; sectorIndex++) {
                    sphere_indices.push_back(0);
                    sphere_indices.push_back(sectorIndex + 2);
                    sphere_indices.push_back(sectorIndex + 1);
                }
                continue;
            }

            if(upPointExists && stackIndex >= stacksLastIndic - 1) {
                // не нужно пытаться генерировать основной меш вверх если вверху уже шапка
                continue;
            }

            if(stackIndex >= stacksLastIndic) {
                // не нужно генерировать меш вверх если выше нету шапки и грани следующей тоже нету
                continue;
            }

            // основной меш
            //int startPoint = (stackIndex - bottomPointExists) * (pointsBySector) + bottomPointExists;
            int startPoint = (stackIndex - bottomPointExists) * pointsBySector + bottomPointExists;
            int nextStartPoint = startPoint + pointsBySector;

            for(int sectorIndex = 0; sectorIndex < sectorsLastCount; sectorIndex++, startPoint++, nextStartPoint++) {
                sphere_indices.push_back(startPoint);
                sphere_indices.push_back(nextStartPoint);
                sphere_indices.push_back(startPoint + 1);

                sphere_indices.push_back(startPoint + 1);
                sphere_indices.push_back(nextStartPoint);
                sphere_indices.push_back(nextStartPoint + 1);
            }
        }
    }

    std::vector<float> sphere_vertices;
    std::vector<float> normals;
    std::vector<float> texCords;
    std::vector<float> unitSquareCoordinates;
    std::vector<unsigned int> sphere_indices;
    std::vector<unsigned int> lineIndices;
};


#endif //TUSA_ANDROID_SPHERE_H
