//
// Created by Artem on 11.05.2024.
//

#ifndef TUSA_ANDROID_TRIANGLE_EXAMPLE_H
#define TUSA_ANDROID_TRIANGLE_EXAMPLE_H

#include <string>
#include <GLES2/gl2.h>
#include "open_gl_interface.h"
#include "cube_example.h"


class TriangleExample : public IOpenGl {
public:
    void render() override;
    void onSurfaceChanged(int w, int h) override;
    void onSurfaceCreated(AAssetManager* assetManager) override;
    void noOpenGlContextInit(AAssetManager* assetManager, float scaleFactor, JNIEnv *env) override;
    void drag(float dx, float dy) override;
    void scale(float scaleFactor) override;
    void doubleTap() override;

public:
    TriangleExample() {
        glVertexShader =
                "attribute vec4 vertexPosition;\n"
                "void main()\n"
                "{\n"
                "    gl_Position = vertexPosition;\n"
                "}\n";

        glFragmentShader =
                "precision mediump float;\n"
                "uniform vec4 fragColour;\n"
                "void main()\n"
                "{\n"
                "    gl_FragColor = fragColour;\n"
                "}\n";
    }

public:
    GLuint vertexLocation;
    GLuint vertexColorLocation;
    GLuint simpleTriangleProgram;
    std::string glVertexShader;
    std::string glFragmentShader;
};


#endif //TUSA_ANDROID_TRIANGLE_EXAMPLE_H
