//
// Created by Artem on 15.02.2024.
//

#ifndef TUSA_FEATURE_GEOMETRY_BLOCK_H
#define TUSA_FEATURE_GEOMETRY_BLOCK_H


template<typename T>
struct FeatureGeometryBlock {
    FeatureGeometryBlock(T* data, size_t size) : data(data), size(size) {

    }

    T* data;
    size_t size;
};

#endif //TUSA_FEATURE_GEOMETRY_BLOCK_H
