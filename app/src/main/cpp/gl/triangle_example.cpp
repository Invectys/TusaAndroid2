//
// Created by Artem on 11.05.2024.
//

#include "gl/triangle_example.h"
#include "util/android_log.h"


void TriangleExample::render() {
    const GLfloat triangleVertices[] = {
            0.0f, 1.0f,
            -1.0f, -1.0f,
            1.0f, -1.0f
    };

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear (GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    glUseProgram(simpleTriangleProgram);
    glVertexAttribPointer(vertexLocation, 2, GL_FLOAT, GL_FALSE, 0 ,triangleVertices);
    glEnableVertexAttribArray(vertexLocation);
    glUniform4f(vertexColorLocation, 1.0f, 1.0f, 1.0f, 1.0f);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

void TriangleExample::onSurfaceChanged(int w, int h) {
    glViewport(0,0,w,h);
}

void TriangleExample::onSurfaceCreated(AAssetManager *assetManager) {

    simpleTriangleProgram = CubeExample::createProgram(glVertexShader.data(), glFragmentShader.data());
    if(simpleTriangleProgram == 0) {
        LOGE("Could not create program");
    }
    vertexLocation = glGetAttribLocation(simpleTriangleProgram, "vertexPosition");
    vertexColorLocation = glGetUniformLocation(simpleTriangleProgram, "fragColour");
}

void TriangleExample::noOpenGlContextInit(AAssetManager *assetManager, float scaleFactor) {

}

void TriangleExample::drag(float dx, float dy) {

}

void TriangleExample::scale(float scaleFactor) {

}

void TriangleExample::doubleTap() {

}
