#include <jni.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <csetjmp>
#include <vector>
#include <math.h>
#include <sstream>
#include <iosfwd>

#include <android/log.h>
#include <cstdlib>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include "util/matrices.h"
#include "util/frustrums.h"
#include "shader/shaders_bucket.h"
#include "map/map.h"
#include "cache/cache.h"
#include "gl/cube_example.h"
#include "gl/triangle_example.h"

#define LOG_TAG "GL_ARTEM"

Cache *cache = new Cache();
Map map = Map(cache);
IOpenGl& openGl = map;


extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_render(JNIEnv *env, jclass clazz) {
    openGl.render();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_onSurfaceChanged(JNIEnv *env, jclass clazz, jint width,
                                                          jint height) {
    openGl.onSurfaceChanged(width, height);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_surfaceCreated(JNIEnv *env, jclass clazz,
                                                        jobject assetManager) {
    AAssetManager* assetManagerFromJava = AAssetManager_fromJava(env, assetManager);
    openGl.onSurfaceCreated(assetManagerFromJava);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_noOpenGlContextInit(JNIEnv *env, jclass clazz,
                                                             jobject assetManager,
                                                             jfloat scale_factor) {
    AAssetManager* assetManagerFromJava = AAssetManager_fromJava(env, assetManager);
    openGl.noOpenGlContextInit(assetManagerFromJava, scale_factor, env);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_setupCacheDirAbsolutePath(JNIEnv *env, jclass clazz,
                                                                   jstring cacheDirAbsolutePath) {
    const char* inputChars = env->GetStringUTFChars(cacheDirAbsolutePath, NULL);
    cache->setAbsoluteCacheDirPath(inputChars);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_drag(JNIEnv *env, jclass clazz, jfloat dx, jfloat dy) {
    openGl.drag(dx, dy);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_scale(JNIEnv *env, jclass clazz, jfloat scale_factor) {
    openGl.scale(scale_factor);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_doubleTap(JNIEnv *env, jclass clazz) {
    openGl.doubleTap();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_onStop(JNIEnv *env, jclass clazz) {
    openGl.onStop();
}