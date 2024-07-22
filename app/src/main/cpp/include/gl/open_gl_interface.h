//
// Created by Artem on 11.05.2024.
//

#ifndef TUSA_ANDROID_OPEN_GL_INTERFACE_H
#define TUSA_ANDROID_OPEN_GL_INTERFACE_H

#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include "network/get_tile_request.h"

class IOpenGl {
public:
    virtual void render() = 0;
    virtual void onSurfaceChanged(int w, int h) = 0;
    virtual void onSurfaceCreated(AAssetManager* assetManagerFromJava) = 0;
    virtual void noOpenGlContextInit(AAssetManager* assetManagerFromJava, float scaleFactor, JNIEnv *env) = 0;
    virtual void drag(float dx, float dy) = 0;
    virtual void scale(float scaleFactor) = 0;
    virtual void doubleTap() = 0;
    virtual void onStop() = 0;
};


#endif //TUSA_ANDROID_OPEN_GL_INTERFACE_H
