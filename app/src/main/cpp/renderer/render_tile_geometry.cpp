//
// Created by Artem on 30.03.2024.
//

#include "renderer/render_tile_geometry.h"


RenderTileGeometry::RenderTileGeometry(std::shared_ptr<ShadersBucket> shadersBucket): shadersBucket(shadersBucket) {

}


void RenderTileGeometry::render(Matrix4 pvm, Matrix4 modelMatrix, Tile *tile) {
    std::shared_ptr<PlainShader> plainShader = shadersBucket->plainShader;
    glUseProgram(plainShader->program);

    glStencilFunc(GL_GREATER, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    // geometryHeap - это куча с геометрией, которую можно отрисовать в тайле единичным вызовом draw метода
    for(short geometryHeapIndex = 0; geometryHeapIndex < Style::maxGeometryHeaps; ++geometryHeapIndex) {
        Matrix4 modelMatrixWithZ = modelMatrix;
        modelMatrixWithZ.translate(0, 0, (Style::maxGeometryHeaps - geometryHeapIndex) * zCordDrawHeapsDiff);
        Matrix4 projectionViewMatrix = pvm * modelMatrixWithZ;
        glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, projectionViewMatrix.get());
        float lineWidth = tile->style.getLineWidthOfHeap(geometryHeapIndex);
        glLineWidth(lineWidth);

        CSSColorParser::Color colorOfStyle = tile->style.getColorOfGeometryHeap(geometryHeapIndex);
        GLfloat red   = static_cast<GLfloat>(colorOfStyle.r) / 255;
        GLfloat green = static_cast<GLfloat>(colorOfStyle.g) / 255;
        GLfloat blue  = static_cast<GLfloat>(colorOfStyle.b) / 255;
        GLfloat alpha = static_cast<GLfloat>(colorOfStyle.a);
        const GLfloat color[] = { red, green, blue, alpha};
        glUniform4fv(plainShader->getColorLocation(), 1, color);

        Geometry<float, unsigned int>& polygonsGeometry = tile->resultPolygons[geometryHeapIndex];
        if(!polygonsGeometry.isEmpty()) {
            glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
            GL_FALSE, 0, polygonsGeometry.points
            );
            glEnableVertexAttribArray(plainShader->getPosLocation());
            glDrawElements(GL_TRIANGLES, polygonsGeometry.indicesCount, GL_UNSIGNED_INT, polygonsGeometry.indices);
        }

        Geometry<float, unsigned int>& linesGeometry = tile->resultLines[geometryHeapIndex];
        if(!linesGeometry.isEmpty()) {
            glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
                                  GL_FALSE, 0, linesGeometry.points
            );
            glEnableVertexAttribArray(plainShader->getPosLocation());
            glDrawElements(GL_LINES, linesGeometry.indicesCount, GL_UNSIGNED_INT, linesGeometry.indices);
        }
    }
}
