//
// Created by Artem on 03.04.2024.
//

#include "renderer/render_tile_coordinates.h"

void RenderTileCoordinates::render(Matrix4 pvmm, VisibleTile visibleTile, int extent) {
    int x = visibleTile.tile->getX();
    int y = visibleTile.tile->getY();
    int z = visibleTile.tile->getZ();

    auto xChar = std::to_string(x);
    auto yChar = std::to_string(y);
    auto zChar = std::to_string(z);

    int frameWidth = 200;
    int framePoints[] = {
            0, 0,
            frameWidth, -frameWidth,
            extent, 0,
            extent - frameWidth, -frameWidth,
            0, -extent,
            frameWidth, -extent + frameWidth,
            extent, -extent,
            extent - frameWidth, -extent + frameWidth
    };

    unsigned short frameIndices[] = {
            0, 1, 2,
            2, 1, 3,
            0, 4, 5,
            0, 1, 5,
            6, 5, 4,
            6, 7, 5,
            6, 7, 3,
            2, 3, 6
    };

    auto plainShader = shadersBucket->plainShader;
    glUseProgram(plainShader->program);
    glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, pvmm.get());

    glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_INT, GL_FALSE, 0, framePoints);
    glEnableVertexAttribArray(plainShader->getPosLocation());

    glUniform4f(plainShader->getColorLocation(), 0.0, 0.0, 0.0f, 1.0f);
    glDrawElements(GL_TRIANGLES, 24, GL_UNSIGNED_SHORT, frameIndices);

    symbols->renderText(xChar + ":" + yChar, extent / 2.0, -extent / 2, pvmm, 2);
}

RenderTileCoordinates::RenderTileCoordinates(
        std::shared_ptr<ShadersBucket> shadersBucket,
        std::shared_ptr<Symbols> symbols
) : shadersBucket(shadersBucket),
    symbols(symbols) {  }



