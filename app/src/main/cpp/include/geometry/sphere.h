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
    void generateSphereData5(int stackCount, int sectorCount,
                             float radius,
                             double centerLatitudeRad,
                             double centerLongitudeRad,
                             double fromCenterLongitudeDeltaRad,
                             double fromCenterLatitudeDeltaRad
    ) {
        // clear memory of prev arrays
        std::vector<float>().swap(sphere_vertices);
        std::vector<unsigned int>().swap(sphere_indices);
        std::vector<float>().swap(normals);
        std::vector<double>().swap(unitSquareCoordinates);

        double sectorStep = 2.0 * fromCenterLongitudeDeltaRad / sectorCount; // Один шаг по longitude
        double stackStep =  2.0 * fromCenterLatitudeDeltaRad / stackCount; // Один шаг по latitude

        centerLongitudeRad = ((int)(centerLongitudeRad / sectorStep) * sectorStep);
        centerLatitudeRad = ((int)(centerLatitudeRad / stackStep) * stackStep);

        fromCenterLongitudeDeltaRad = ((int)(fromCenterLongitudeDeltaRad / sectorStep) * sectorStep);
        fromCenterLatitudeDeltaRad = ((int)(fromCenterLatitudeDeltaRad / stackStep) * stackStep);

        bool bottomPointExists = false;
        bool upPointExists = false;
        int seamIndex = -1;

        for(int stackIndex = 0; stackIndex <= stackCount; stackIndex++) {
            double stackRad = CommonUtils::clamp(centerLatitudeRad - fromCenterLatitudeDeltaRad + stackStep * stackIndex, -M_PI / 2, M_PI / 2); // текущий latitude
            double latitude = stackRad; //    - PI / 2 to PI / 2

            // Это верхняя или нижняя точка
            bool isBottomPoint = stackIndex == 0 && latitude == -M_PI / 2;
            bool isUpPoint = stackIndex == stackCount && latitude == M_PI / 2;
            if (isBottomPoint)
                bottomPointExists = true;
            if (isUpPoint) {
                upPointExists = true;
            }

            for(int sectorIndex = 0; sectorIndex <= sectorCount; sectorIndex++) {
                double sectorRad = CommonUtils::normalizeLongitudeRad(centerLongitudeRad - fromCenterLongitudeDeltaRad + sectorStep * sectorIndex) + M_PI;

                float y = radius * sinf(stackRad);
                float x = radius * cosf(stackRad) * cosf(sectorRad);
                float z = radius * cosf(stackRad) * sinf(sectorRad);
                double s = CommonUtils::latitudeRadToY(latitude); // это координата тексуты карты вдоль latitude

                float epsilon = sectorStep / 5;
                bool isSeam = CommonUtils::compareFloats(sectorRad, 2 * M_PI, epsilon) ||
                              CommonUtils::compareFloats(sectorRad, 0, epsilon);

                if (isSeam && sectorIndex != 0) {
                    seamIndex = sectorIndex;
                    sphere_vertices.push_back(x);
                    sphere_vertices.push_back(y);
                    sphere_vertices.push_back(z);

                    unitSquareCoordinates.push_back(0); // Это конец карты, текстуры вдоль longitude
                    unitSquareCoordinates.push_back(s);

                    sphere_vertices.push_back(x);
                    sphere_vertices.push_back(y);
                    sphere_vertices.push_back(z);

                    unitSquareCoordinates.push_back(1); // Это начало карты, текстуры вдоль longitude
                    unitSquareCoordinates.push_back(s);
                } else {
                    sphere_vertices.push_back(x);
                    sphere_vertices.push_back(y);
                    sphere_vertices.push_back(z);

                    double t = CommonUtils::clamp(1.0 - sectorRad / (2.0 * M_PI), 0.0, 1.0); // координата текстуры вдоль longitude
                    unitSquareCoordinates.push_back(t);
                    unitSquareCoordinates.push_back(s);
                }

                if(isUpPoint || isBottomPoint) {
                    break;
                }
            }
        }

        bool centerSeamExists = seamIndex != -1; // есть ли шов где-то по центру отображаемой зоны.

        int stacksLastIndic = stackCount;
        int fullRings = stacksLastIndic + 1 - upPointExists - bottomPointExists; // количество колец без учета нижней и верхней шапки
        int pointsBySector = sectorCount + centerSeamExists + 1; // сколько точек вдоль longitude есть +1 так как включительно <= лупом проходим
        int sectorsLastCount = sectorCount;

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
            int startPoint = (stackIndex - bottomPointExists) * pointsBySector + bottomPointExists; // начальная точка первого кольца
            int nextStartPoint = startPoint + pointsBySector; // верхний сосед кольца


            for(int sectorIndex = 0; sectorIndex < sectorsLastCount + centerSeamExists; sectorIndex++, startPoint++, nextStartPoint++) {
                if (sectorIndex == seamIndex)
                    continue;
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
    std::vector<double> unitSquareCoordinates;
    std::vector<unsigned int> sphere_indices;
    std::vector<unsigned int> lineIndices;
};


#endif //TUSA_ANDROID_SPHERE_H
