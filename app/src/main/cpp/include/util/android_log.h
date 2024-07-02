//
// Created by Artem on 07.02.2024.
//

#ifndef TUSA_ANDROID_LOG_H
#define TUSA_ANDROID_LOG_H

#include <math.h>
#include <cmath>
#include <vector>
#include "map/mercator.h"
#include <android/log.h>
#include <GLES2/gl2.h>
#include <string>
#define LOG_TAG "GL_ARTEM"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

class CommonUtils {
public:
    static void printGlError() {
        LOGI("Open gl error %s", getGLErrorString().c_str());
    }

    static std::string getGLErrorString() {
        switch (glGetError()) {
            case GL_NO_ERROR: return "GL_NO_ERROR";
            case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
            case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
            case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
            case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
            case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
            default: return "UNKNOWN_ERROR";
        }
    }

    // преобразует угол в радианах в позицию от 0 до 1 сферы
    static float latitudeRadToY(float latitudeRad) {
        float lowerStackRad = (float) DEG2RAD(-degLatitudeConstraint);
        float highStackRad = (float) DEG2RAD(degLatitudeConstraint);
        float stackRadClipped = std::max(lowerStackRad, std::min(latitudeRad, highStackRad));
        float stackCord = (std::logf(std::tanf(stackRadClipped) + 1.0f / std::cosf(stackRadClipped)));
        float stackCord_n = 0.5 - stackCord / (2.0f * M_PI);
        float y = std::min(std::max(0.0f, 1.0f - stackCord_n), 1.0f);
        return y;
    }

    // longitudeRad should be in -M_PI to M_PI
    static float longitudeRadToX(float longitudeRad) {
        return 0.5 + RAD2DEG(longitudeRad) / 360;
    }

    static float fract(float value) {
        return value - std::floor(value);
    }
private:
    //constexpr static const float degLatitudeConstraint = 85.0511f;
    constexpr static const float degLatitudeConstraint = 89.9f;
};

#endif //TUSA_ANDROID_LOG_H
