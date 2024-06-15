//
// Created by Artem on 11.05.2024.
//


#include <malloc.h>
#include "gl/cube_example.h"
#include "util/android_log.h"
#include "util/frustrums.h"

void CubeExample::render() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    glUseProgram(shadersBucket.plainShader->program);
    modelViewMatrix = Matrix4();
    modelViewMatrix.translate(0.0f, 0.0f, -10.0f);
    glVertexAttribPointer(shadersBucket.plainShader->getPosLocation(), 3, GL_FLOAT, GL_FALSE, 0, triangleVertices);
    glEnableVertexAttribArray(vertexLocation);

    Matrix4 matrix = projectionMatrix * modelViewMatrix;
    glUniformMatrix4fv(shadersBucket.plainShader->getMatrixLocation(), 1, GL_FALSE, matrix.get());
    glUniform4f(shadersBucket.plainShader->getColorLocation(), 1.0f, 1.0f, 1.0f, 1.0f);
    glDrawArrays(GL_TRIANGLES, 0, 3);


//    modelViewMatrix = Matrix4();
//    //modelViewMatrix.rotateX(angle);
//    modelViewMatrix.translate(0.0f, 0.0f, -10.0f);
//    Matrix4 matrix = projectionMatrix * modelViewMatrix;
//
//    glUseProgram(shadersBucket.plainShader->program);
//    glVertexAttribPointer(shadersBucket.plainShader->getPosLocation(), 3, GL_FLOAT, GL_FALSE, 0, triangleVertices);
//    glEnableVertexAttribArray(shadersBucket.plainShader->getPosLocation());

//    glVertexAttribPointer(shadersBucket.plainShader->getColorLocation(), 3, GL_FLOAT, GL_FALSE, 0, colour);
//    glEnableVertexAttribArray(shadersBucket.plainShader->getColorLocation());
//    glUniform4f(shadersBucket.plainShader->getColorLocation(), 1.0f, 1.0f, 1.0f, 1.0f);

//    glUniformMatrix4fv(shadersBucket.plainShader->getMatrixLocation(), 1, GL_FALSE, matrix.get());
    //glUniformMatrix4fv(modelViewLocation, 1, GL_FALSE, modelViewMatrix.get());
    //glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, indices);
//    glDrawArrays(GL_TRIANGLES, 0, 3);

    if(angle > 360) {
        angle = 0;
    }
    angle++;
}

void CubeExample::onSurfaceChanged(int w, int h) {
    simpleTriangleProgram = createProgram(glVertexShader.data(), glFragmentShader.data());
    if(simpleTriangleProgram == 0) {
        LOGE("Could not create program");
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
}

void CubeExample::onSurfaceCreated(AAssetManager *assetManager) {
    shadersBucket.compileAllShaders(assetManager);
}

void CubeExample::noOpenGlContextInit(AAssetManager *assetManager, float scaleFactor) {

}

void CubeExample::drag(float dx, float dy) {

}

void CubeExample::scale(float scaleFactor) {

}

void CubeExample::doubleTap() {

}
