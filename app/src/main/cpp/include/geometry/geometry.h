//
// Created by Artem on 15.02.2024.
//

#ifndef TUSA_GEOMETRY_H
#define TUSA_GEOMETRY_H

template <class Coord, class Indic>
class Geometry {
public:
    Geometry(Coord* points, size_t pointsCount, Indic* indices, size_t indicesCount);
    Geometry();

    Coord *points = nullptr;
    Indic *indices = nullptr;
    size_t pointsCount = 0;
    size_t indicesCount = 0;

    bool isEmpty() { return pointsCount == 0 || indicesCount == 0; }
};

template<class Coord, class Indic>
Geometry<Coord, Indic>::Geometry() {

}

template<class Coord, class Indic>
Geometry<Coord, Indic>::Geometry(Coord *points, size_t pointsCount, Indic *indices, size_t indicesCount) :
                                 points(points), pointsCount(pointsCount),
                                 indicesCount(indicesCount), indices(indices)  {

}


#endif //TUSA_GEOMETRY_H
