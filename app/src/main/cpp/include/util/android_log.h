//
// Created by Artem on 07.02.2024.
//

#ifndef TUSA_ANDROID_LOG_H
#define TUSA_ANDROID_LOG_H

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
};

#endif //TUSA_ANDROID_LOG_H
