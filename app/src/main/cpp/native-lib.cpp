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


#define LOG_TAG "GL_ARTEM"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

GLuint simpleTriangleProgram;
GLuint vPosition;
GLuint vertexLocation;
GLuint vertexColorLocation;
GLuint projectionLocation;
GLuint modelViewLocation;
Matrix4 projectionMatrix;
Matrix4 modelViewMatrix;
float angle = 0;
Cache *cache = new Cache();
Map map = Map(cache);


GLuint loadShader(GLenum shaderType, const char* shaderSource) {
    GLuint shader = glCreateShader(shaderType);
    if(shader) {
        glShaderSource(shader, 1, &shaderSource, NULL);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if(!compiled) {
            GLint infoLen = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
            if(infoLen) {
                char* buf = (char*) malloc(infoLen);
                if(buf) {
                    glGetShaderInfoLog(shader, infoLen, NULL, buf);
                    LOGE("Could not compile shader %d:\n%s\n", shaderType, buf);
                    free(buf);
                }
                glDeleteShader(shader);
                shader = 0;
            }
        }
    }
    return shader;
}

GLuint createProgram(const char* vertexSource, const char* fragmentSource) {
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER, vertexSource);
    if(!vertexShader) {
        return 0;
    }
    GLuint fragmentShader = loadShader(GL_FRAGMENT_SHADER, fragmentSource);
    if(!fragmentShader) {
        return 0;
    }
    GLuint program = glCreateProgram();
    if(program) {
        glAttachShader(program, vertexShader);
        glAttachShader(program, fragmentShader);
        glLinkProgram(program);
        GLint linkStatus = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
        if(linkStatus != GL_TRUE) {
            GLint bufLength = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufLength);
            if(bufLength) {
                char* buf = (char *) malloc(bufLength);
                if(buf) {
                    glGetProgramInfoLog(program, bufLength, NULL, buf);
                    LOGE("Could not link program:\n%s\n", buf);
                    free(buf);
                }
            }
            glDeleteProgram(program);
            program = 0;
        }
    }
    return program;
}

static const char  glVertexShader[] =
        "attribute vec4 vertexPosition;\n"
        "attribute vec3 vertexColour;\n"
        "varying vec3 fragColour;\n"
        "uniform mat4 projection;\n"
        "uniform mat4 modelView;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = projection * modelView * vertexPosition;\n"
        "    fragColour = vertexColour;\n"
        "}\n";

static const char glFragmentShader[] =
        "precision mediump float;\n"
        "varying vec3 fragColour;\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = vec4(fragColour, 1.0);\n"
        "}\n";

const GLfloat triangleVertices[] = {
        0.0f, 1.0f,
        -1.0f, -1.0f,
        1.0f, -1.0f
};

GLfloat colour[] = {1.0f, 0.0f, 0.0f,
                    1.0f, 0.0f, 0.0f,
                    1.0f, 0.0f, 0.0f,
                    1.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 1.0f,
                    0.0f, 0.0f, 1.0f,
                    0.0f, 0.0f, 1.0f,
                    0.0f, 0.0f, 1.0f,
                    1.0f, 1.0f, 0.0f,
                    1.0f, 1.0f, 0.0f,
                    1.0f, 1.0f, 0.0f,
                    1.0f, 1.0f, 0.0f,
                    0.0f, 1.0f, 1.0f,
                    0.0f, 1.0f, 1.0f,
                    0.0f, 1.0f, 1.0f,
                    0.0f, 1.0f, 1.0f,
                    1.0f, 0.0f, 1.0f,
                    1.0f, 0.0f, 1.0f,
                    1.0f, 0.0f, 1.0f,
                    1.0f, 0.0f, 1.0f
};

const GLfloat cubeVertices[] = {-1.0f,  1.0f, -1.0f, /* Back. */
                                1.0f,  1.0f, -1.0f,
                                -1.0f, -1.0f, -1.0f,
                                1.0f, -1.0f, -1.0f,
                                -1.0f,  1.0f,  1.0f, /* Front. */
                                1.0f,  1.0f,  1.0f,
                                -1.0f, -1.0f,  1.0f,
                                1.0f, -1.0f,  1.0f,
                                -1.0f,  1.0f, -1.0f, /* Left. */
                                -1.0f, -1.0f, -1.0f,
                                -1.0f, -1.0f,  1.0f,
                                -1.0f,  1.0f,  1.0f,
                                1.0f,  1.0f, -1.0f, /* Right. */
                                1.0f, -1.0f, -1.0f,
                                1.0f, -1.0f,  1.0f,
                                1.0f,  1.0f,  1.0f,
                                -1.0f, -1.0f, -1.0f, /* Top. */
                                -1.0f, -1.0f,  1.0f,
                                1.0f, -1.0f,  1.0f,
                                1.0f, -1.0f, -1.0f,
                                -1.0f,  1.0f, -1.0f, /* Bottom. */
                                -1.0f,  1.0f,  1.0f,
                                1.0f,  1.0f,  1.0f,
                                1.0f,  1.0f, -1.0f
};

GLushort indices[] = {0, 2, 3, 0, 1, 3, 4, 6, 7, 4, 5, 7, 8, 9, 10, 11, 8, 10, 12, 13, 14, 15, 12, 14, 16, 17, 18, 16, 19, 18, 20, 21, 22, 20, 23, 22};

void renderFrame() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);


    modelViewMatrix = Matrix4();
    modelViewMatrix.rotateX(angle);
    modelViewMatrix.translate(0.0f, 0.0f, -10.0f);

    glUseProgram(simpleTriangleProgram);
    glVertexAttribPointer(vertexLocation, 3, GL_FLOAT, GL_FALSE, 0, cubeVertices);
    glEnableVertexAttribArray(vertexLocation);

    glVertexAttribPointer(vertexColorLocation, 3, GL_FLOAT, GL_FALSE, 0, colour);
    glEnableVertexAttribArray(vertexColorLocation);

    glUniformMatrix4fv(projectionLocation, 1, GL_FALSE, projectionMatrix.get());
    glUniformMatrix4fv(modelViewLocation, 1, GL_FALSE, modelViewMatrix.get());
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, indices);

    if(angle > 360) {
        angle = 0;
    }
    angle++;
}

bool setupGraphics(int w, int h) {
    simpleTriangleProgram = createProgram(glVertexShader, glFragmentShader);
    if(simpleTriangleProgram == 0) {
        LOGE("Could not create program");
        return false;
    }
    vertexLocation = glGetAttribLocation(simpleTriangleProgram, "vertexPosition");
    vertexColorLocation = glGetAttribLocation(simpleTriangleProgram, "vertexColour");
    projectionLocation = glGetUniformLocation(simpleTriangleProgram, "projection");
    modelViewLocation = glGetUniformLocation(simpleTriangleProgram, "modelView");


    //llmr::matrix::ortho(projectionMatrix, -10, 10, -10, 10, 0.1f, 100);
    projectionMatrix = setFrustum(45, (float) w / (float) h, 0.1f, 100);
    //matrixPerspective(projectionMatrix, 45, (float) w / (float) h, 0.1f, 100);
    glEnable(GL_DEPTH_TEST);
    glViewport(0,0,w,h);
    return true;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_render(JNIEnv *env, jclass clazz) {
    map.render();
    //renderFrame();
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_onSurfaceChanged(JNIEnv *env, jclass clazz, jint width,
                                                          jint height) {
    //setupGraphics(width, height);
    map.onSurfaceChanged(width, height);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_surfaceCreated(JNIEnv *env, jclass clazz,
                                                        jobject assetManager) {
    AAssetManager* assetManagerFromJava = AAssetManager_fromJava(env, assetManager);
    map.onSurfaceCreated(assetManagerFromJava);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_noOpenGlContextInit(JNIEnv *env, jclass clazz,
                                                             jobject assetManager,
                                                             jfloat scale_factor) {
    AAssetManager* assetManagerFromJava = AAssetManager_fromJava(env, assetManager);
    map.noOpenGlContextInit(assetManagerFromJava, scale_factor);
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
    map.drag(dx, dy);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_scale(JNIEnv *env, jclass clazz, jfloat scale_factor) {
    map.scale(scale_factor);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_artem_tusaandroid_NativeLibrary_doubleTap(JNIEnv *env, jclass clazz) {
    map.doubleTap();
}
