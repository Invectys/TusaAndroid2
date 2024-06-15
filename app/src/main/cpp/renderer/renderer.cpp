//
// Created by Artem on 02.01.2024.
//

#include "renderer/renderer.h"
#include <GLES2/gl2.h>
#include "util/matrices.h"
#include "util/frustrums.h"
#include <thread>
#include <mapbox/earcut.hpp>
#include "util/android_log.h"
#include "shader/planet_shader.h"
#include <cmath>
#include <iostream>
#include <thread>
#include <stack>

bool DEBUG_TILE = false;
bool UPDATE_ON_DRAG = true;

std::mutex evaluateTilesPositionsMutex;

Renderer::Renderer(
        std::shared_ptr<ShadersBucket> shadersBucket,
        Cache* cache
):
    shadersBucket(shadersBucket),
    cache(cache),
    renderTileGeometry(RenderTileGeometry(shadersBucket)) {
    symbols = std::shared_ptr<Symbols>(new Symbols(shadersBucket));
    renderTileCoordinates = std::shared_ptr<RenderTileCoordinates>(new RenderTileCoordinates(shadersBucket, symbols));
}

float size = 0;

void Renderer::renderFrame() {
    evaluateTilesPositionsMutex.lock();
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<PlainShader> plainShader = shadersBucket->plainShader;
    std::shared_ptr<SymbolShader> symbolShader = shadersBucket->symbolShader;
    std::shared_ptr<PlanetShader> planetShader = shadersBucket->planetShader;

    glStencilMask(0xFF);
    glClearColor((float)242 / 255, (float)248 / 255, (float)230 / 255, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glStencilFunc(GL_GREATER, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glUseProgram(plainShader->program);

    std::set<short> possibleDeltasSet = {};
    std::vector<short> possibleDeltasVec = {};
    for(short renderTile = 0; renderTile < rendererTilesSize; renderTile++) {
        TileForRenderer visibleTile = tilesForRenderer[0];
        if(visibleTile.isEmpty())
            return;
        if(possibleDeltasSet.insert(visibleTile.zDeltaFlag).second) {
            possibleDeltasVec.push_back(visibleTile.zDeltaFlag);
        }
    }

    Matrix4 viewMatrixTexture = Matrix4();
    viewMatrixTexture.translate(0, 0, -1);
    Matrix4 pvmTexture = rendererTileProjectionMatrix * viewMatrixTexture;

    short amountToRender = 0;
    for(short renderTileIndex = 0; renderTileIndex < rendererTilesSize; renderTileIndex++) {
        TileForRenderer visibleTile = tilesForRenderer[0];
        if(!visibleTile.isEmpty())
            amountToRender++;
    }

    GLuint renderTextureIndex = renderTexture[0];
    glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderTextureIndex, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer);

    short reserved = 1;
    for(short zDelta : possibleDeltasVec) {
        for(short geometryHeapIndex = 0; geometryHeapIndex < Style::maxGeometryHeaps + reserved; ++geometryHeapIndex) {
            for(short renderTileIndex = 0; renderTileIndex < rendererTilesSize; renderTileIndex++) {
                TileForRenderer visibleTile = tilesForRenderer[renderTileIndex];
                if(visibleTile.isEmpty())
                    continue;
                GLuint renderTextureCurrent = renderTexture[renderTileIndex];

                short deltaX = visibleTile.tileX - *rootTileX;
                short deltaY = visibleTile.tileY - *rootTileY;

                short shift = 4096;
                Matrix4 model = Matrix4();
                model.translate(shift * deltaX, -1 * shift * deltaY, 0);
                Matrix4 forTileMatrix = pvmTexture * model;

                if(visibleTile.isEmpty() || visibleTile.zDeltaFlag != zDelta)
                    continue;
                Tile* tile = visibleTile.tile;
                float renderTileScaleFactor = visibleTile.mapScaleFactor;
                glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, forTileMatrix.get());

                // Рисуем основную карту
                if(geometryHeapIndex < Style::maxGeometryHeaps) {
                    Geometry<float, unsigned int>& polygonsGeometry = tile->resultPolygons[geometryHeapIndex];
                    Geometry<float, unsigned int>& linesGeometry = tile->resultLines[geometryHeapIndex];
                    if(polygonsGeometry.isEmpty() && linesGeometry.isEmpty())
                        continue;

                    float lineWidth = tile->style.getLineWidthOfHeap(geometryHeapIndex);
                    glLineWidth(lineWidth);

                    CSSColorParser::Color colorOfStyle = tile->style.getColorOfGeometryHeap(geometryHeapIndex);
                    GLfloat red   = static_cast<GLfloat>(colorOfStyle.r) / 255;
                    GLfloat green = static_cast<GLfloat>(colorOfStyle.g) / 255;
                    GLfloat blue  = static_cast<GLfloat>(colorOfStyle.b) / 255;
                    GLfloat alpha = static_cast<GLfloat>(colorOfStyle.a);
                    const GLfloat color[] = { red, green, blue, alpha};
                    glUniform4fv(plainShader->getColorLocation(), 1, color);


                    if(!polygonsGeometry.isEmpty()) {
                        glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
                                              GL_FALSE, 0, polygonsGeometry.points
                        );
                        glEnableVertexAttribArray(plainShader->getPosLocation());
                        glDrawElements(GL_TRIANGLES, polygonsGeometry.indicesCount, GL_UNSIGNED_INT, polygonsGeometry.indices);
                    }

                    if(!linesGeometry.isEmpty()) {
                        glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
                                              GL_FALSE, 0, linesGeometry.points
                        );
                        glEnableVertexAttribArray(plainShader->getPosLocation());
                        glDrawElements(GL_LINES, linesGeometry.indicesCount, GL_UNSIGNED_INT, linesGeometry.indices);
                    }

                    continue;
                }

                // рисуем бекграунд
                if(geometryHeapIndex == Style::maxGeometryHeaps) {
                    CSSColorParser::Color colorOfStyle = CSSColorParser::parse("rgb(241, 255, 230)");
                    GLfloat red   = static_cast<GLfloat>(colorOfStyle.r) / 255;
                    GLfloat green = static_cast<GLfloat>(colorOfStyle.g) / 255;
                    GLfloat blue  = static_cast<GLfloat>(colorOfStyle.b) / 255;
                    GLfloat alpha = static_cast<GLfloat>(colorOfStyle.a);
                    const GLfloat color[] = { red, green, blue, alpha};
                    glUniform4fv(plainShader->getColorLocation(), 1, color);

                    float backPoints[] = {
                            0, 0,
                            (float) extent, 0,
                            (float) extent, -(float) extent,
                            0, 0,
                            0, - (float) extent,
                            (float) extent, -(float) extent,
                    };

                    glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
                                          GL_FALSE, 0, backPoints
                    );
                    glEnableVertexAttribArray(plainShader->getPosLocation());
                    glDrawArrays(GL_TRIANGLES, 0, 6);

                }
            }
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glStencilMask(0x0);

    CommonUtils::printGlError();
    glUseProgram(planetShader->program);
    float points[] = {
            0.0f, 0.0f,
            4096.0f, 0.0f,
            4096.0f, -4096.0f,
            0.0f, -4096.0f
    };
    float texCords[] = {
            0, 0,
            1, 0,
            0, 1,
            1, 1
    };
    glUniformMatrix4fv(planetShader->getMatrixLocation(), 1, GL_FALSE, pvmTexture.get());
    glVertexAttribPointer(planetShader->getPosLocation(), 2, GL_FLOAT, GL_FALSE, 0, points);
    glEnableVertexAttribArray(planetShader->getPosLocation());
    glVertexAttribPointer(planetShader->getUnitSquareCoordinates(), 2, GL_FLOAT, GL_FALSE, 0, texCords);
    glEnableVertexAttribArray(planetShader->getUnitSquareCoordinates());
    glBindTexture(GL_TEXTURE_2D, renderTexture[0]);
    glUniform1i(planetShader->getTileTextureLocation(0), 0);
    unsigned short indices[] = {
            0, 3, 1,
            1, 3, 2
    };
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices);
    CommonUtils::printGlError();

    return;
    drawPlanet();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    //LOGI("render time: %f", duration.count());w
    evaluateTilesPositionsMutex.unlock();
}

void Renderer::drawPlanet() {
    std::shared_ptr<PlanetShader> planetShader = shadersBucket->planetShader;
    Matrix4 viewMatrix = Matrix4();
    viewMatrix.rotate(cameraXAngleDeg, 0, 1, 0);
    viewMatrix.rotate(cameraYAngleDeg, 1, 0, 0);
    viewMatrix.translate(0, 0, camera[2]);
    Matrix4 pvm = projectionMatrix * viewMatrix;
    Matrix4 modelMatrix = Matrix4();
    modelMatrix.rotateY(90);
    Matrix4 pvmm = pvm * modelMatrix;
    glUseProgram(planetShader->program);

    glUniformMatrix4fv(planetShader->getMatrixLocation(), 1, GL_FALSE, pvmm.get());
    glUniform4f(planetShader->getColorLocation(), 0.0f, 0.0f, 0.0f, 1.0f);
    glVertexAttribPointer(planetShader->getUnitSquareCoordinates(), 2, GL_FLOAT, GL_FALSE, 0, sphere.unitSquareCoordinates.data());
    glEnableVertexAttribArray(planetShader->getUnitSquareCoordinates());
    glVertexAttribPointer(planetShader->getPosLocation(), 3, GL_FLOAT, GL_FALSE, 0, sphere.sphere_vertices.data());
    glEnableVertexAttribArray(planetShader->getPosLocation());


    int d = GL_TEXTURE0;
    int activeTextureUnit = d + 0;
    glActiveTexture(activeTextureUnit);
    glBindTexture(GL_TEXTURE_2D, renderTexture[0]);
    glUniform1i(planetShader->getTileTextureLocation(0), 0);
    glVertexAttribPointer(planetShader->getTileTextureCoordinate(0), 2, GL_FLOAT, GL_FALSE, 0, cord_tiles[0].data());
    glEnableVertexAttribArray(planetShader->getTileTextureCoordinate(0));

    //glDrawElements(GL_LINE_LOOP, sphere.sphere_indices.size(), GL_UNSIGNED_INT, sphere.sphere_indices.data());
    glDrawElements(GL_TRIANGLES, sphere.sphere_indices.size(), GL_UNSIGNED_INT, sphere.sphere_indices.data());

}

void Renderer::drawTile(TileForRenderer visibleTile, Matrix4 pvmTexture, std::vector<short> possibleDeltasVec) {
    std::shared_ptr<PlainShader> plainShader = shadersBucket->plainShader;
    short reserved = 1;
    for(short zDelta : possibleDeltasVec) {
        for(short geometryHeapIndex = 0; geometryHeapIndex < Style::maxGeometryHeaps + reserved; ++geometryHeapIndex) {
            if(visibleTile.isEmpty() || visibleTile.zDeltaFlag != zDelta)
                continue;
            Tile* tile = visibleTile.tile;
            float renderTileScaleFactor = visibleTile.mapScaleFactor;
            glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, pvmTexture.get());

            // Рисуем основную карту
            if(geometryHeapIndex < Style::maxGeometryHeaps) {
                Geometry<float, unsigned int>& polygonsGeometry = tile->resultPolygons[geometryHeapIndex];
                Geometry<float, unsigned int>& linesGeometry = tile->resultLines[geometryHeapIndex];
                if(polygonsGeometry.isEmpty() && linesGeometry.isEmpty())
                    continue;

                float lineWidth = tile->style.getLineWidthOfHeap(geometryHeapIndex);
                glLineWidth(lineWidth);

                CSSColorParser::Color colorOfStyle = tile->style.getColorOfGeometryHeap(geometryHeapIndex);
                GLfloat red   = static_cast<GLfloat>(colorOfStyle.r) / 255;
                GLfloat green = static_cast<GLfloat>(colorOfStyle.g) / 255;
                GLfloat blue  = static_cast<GLfloat>(colorOfStyle.b) / 255;
                GLfloat alpha = static_cast<GLfloat>(colorOfStyle.a);
                const GLfloat color[] = { red, green, blue, alpha};
                glUniform4fv(plainShader->getColorLocation(), 1, color);


                if(!polygonsGeometry.isEmpty()) {
                    glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
                                          GL_FALSE, 0, polygonsGeometry.points
                    );
                    glEnableVertexAttribArray(plainShader->getPosLocation());
                    glDrawElements(GL_TRIANGLES, polygonsGeometry.indicesCount, GL_UNSIGNED_INT, polygonsGeometry.indices);
                }

                if(!linesGeometry.isEmpty()) {
                    glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
                                          GL_FALSE, 0, linesGeometry.points
                    );
                    glEnableVertexAttribArray(plainShader->getPosLocation());
                    glDrawElements(GL_LINES, linesGeometry.indicesCount, GL_UNSIGNED_INT, linesGeometry.indices);
                }

                continue;
            }

            // рисуем бекграунд
            if(geometryHeapIndex == Style::maxGeometryHeaps) {
                CSSColorParser::Color colorOfStyle = CSSColorParser::parse("rgb(241, 255, 230)");
                GLfloat red   = static_cast<GLfloat>(colorOfStyle.r) / 255;
                GLfloat green = static_cast<GLfloat>(colorOfStyle.g) / 255;
                GLfloat blue  = static_cast<GLfloat>(colorOfStyle.b) / 255;
                GLfloat alpha = static_cast<GLfloat>(colorOfStyle.a);
                const GLfloat color[] = { red, green, blue, alpha};
                glUniform4fv(plainShader->getColorLocation(), 1, color);

                float backPoints[] = {
                        0, 0,
                        (float) extent, 0,
                        (float) extent, -(float) extent,
                        0, 0,
                        0, - (float) extent,
                        (float) extent, -(float) extent,
                };

                glVertexAttribPointer(plainShader->getPosLocation(), 2, GL_FLOAT,
                                      GL_FALSE, 0, backPoints
                );
                glEnableVertexAttribArray(plainShader->getPosLocation());
                glDrawArrays(GL_TRIANGLES, 0, 6);

            }
        }
    }
}

void Renderer::onSurfaceChanged(int w, int h) {
    screenW = w;
    screenH = h;
    updateFrustum();

    for(short renderTextureIndex = 0; renderTextureIndex < rendererTilesSize; renderTextureIndex++) {
        GLuint renderTexturePtr = renderTexture[renderTextureIndex];
        glBindTexture(GL_TEXTURE_2D, renderTexturePtr);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, screenW, screenH, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    }

    glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, screenW, screenH);
}

void Renderer::updateVisibleTiles(bool render) {
    if(DEBUG_TILE) {
        currentVisibleTiles = {
                TileCords {1, 0, 1, 0, 1},
                TileCords {1, 1, 1, 1, 1},
                TileCords {0, 0, 0, 0, 0},
        };
        loadAndRenderCurrentVisibleTiles();
        return;
    }

    std::vector<TileCords> newVisibleTiles = {};
//    auto topCords = evaluateTileCords(0, 0);
//    auto bottomCords = evaluateTileCords(0, 1);
//    if(!topCords.wasNormalized) newVisibleTiles.push_back(topCords);
//    if(!bottomCords.wasNormalized) newVisibleTiles.push_back(bottomCords);
//
//    RenderTilesEnum renderTilesEnum = evaluateRenderTilesCameraXType();
//    if(renderTilesEnum == RenderTilesEnum::SHOW_RIGHT) {
//        LOGI("Show tiles direction: SHOW_RIGHT");
//        auto topRightCords = evaluateTileCords(1, 0);
//        auto bottomRightCords = evaluateTileCords(1, 1);
//
//        if(!topRightCords.wasNormalized) newVisibleTiles.push_back(topRightCords);
//        if(!bottomRightCords.wasNormalized) newVisibleTiles.push_back(bottomRightCords);
//    } else if(renderTilesEnum == RenderTilesEnum::SHOW_LEFT) {
//        LOGI("Show tiles direction: SHOW_LEFT");
//        auto topLeftCords = evaluateTileCords(-1, 0);
//        auto bottomLeftCords = evaluateTileCords(-1, 1);
//
//        if(!topLeftCords.wasNormalized) newVisibleTiles.push_back(topLeftCords);
//        if(!bottomLeftCords.wasNormalized) newVisibleTiles.push_back(bottomLeftCords);
//    }

    if(currentMapZTile() == 0) {
        newVisibleTiles.push_back(evaluateTileCords(0, 0));
    }

    if(currentMapZTile() == 1) {
        newVisibleTiles.push_back(evaluateTileCords(0, 0));
        newVisibleTiles.push_back(evaluateTileCords(0, 1));
        newVisibleTiles.push_back(evaluateTileCords(1, 0));
        newVisibleTiles.push_back(evaluateTileCords(1, 1));
    }

    currentVisibleTiles = std::move(newVisibleTiles);
    toShowTilesQueue.clear();
    if(render)
        loadAndRenderCurrentVisibleTiles();
}

int Renderer::normalizeCoordinate(int coordinate) {
    int sideSize = pow(2, currentMapZTile());
    int tilesAxisLastIndex = sideSize - 1;
    if(tilesAxisLastIndex == 0) {
        return 0;
    }
    if(coordinate >= 0) {
        return coordinate % sideSize;
    }

    coordinate *= -1;
    coordinate %= sideSize;
    if(coordinate == 0) {
        return 0;
    }

    return sideSize - coordinate;
}

void Renderer::drag(float dx, float dy) {
    float scaleForDrag = evaluateFloatScaleFactorFormula();
    cameraYAngleDeg -= dy * dragFingerMapSpeed * scaleForDrag;
    cameraYAngleDeg = std::max(-degLatitudeCameraConstraint, std::min(cameraYAngleDeg, degLatitudeCameraConstraint));
    cameraXAngleDeg -= dx * dragFingerMapSpeed * scaleForDrag;
    LOGI("Planet coordinates: Latitude: %f Longitude %f", getPlanetCurrentLatitude(), getPlanetCurrentLongitude());

    if (!UPDATE_ON_DRAG)
        return;

    auto rootTileXEv = evaluateRootTileDiffX();
    auto rootTileYEv = evaluateRootTileDiffY();

    bool needUpdateVisibleTiles = false;
    if(updateSavedLastDragXTileType()) {
        needUpdateVisibleTiles = true;
    }

    if(rootTileXEv != *rootTileX || rootTileYEv != *rootTileY) {
        //updateRootTile();
        needUpdateVisibleTiles = true;
    }

//    if(needUpdateVisibleTiles) {
//        updateVisibleTiles();
//    }
}

void Renderer::scale(float factor) {
    updateMapZoomScaleFactor(factor); // обновляет текущий скейл, паарметр скейла
    LOGI("Scale factor %f", scaleFactorZoom);

    //updateMapCameraZPosition(); // дистанция камеры от центра
    if(_savedLastScaleStateMapZ != currentMapZTile()) {
        // Новый зум у карты. Новая координата z
        renderTileGeometry.scaleZCordDrawHeapsDiff(evaluateScaleFactorFormula());
        //updateFrustum();
        //updateRootTile();
        //updateSavedLastDragXTileType();
        updateVisibleTiles();
        _savedLastScaleStateMapZ = currentMapZTile();
        LOGI("New Z of map %d", currentMapZTile());
        LOGI("Current tile extent %f", evaluateCurrentExtent());
    }
}

void Renderer::doubleTap() {
    if(visibleTileRenderMode == VisibleTileRenderMode::TILE_COORDINATES) {
        visibleTileRenderMode = VisibleTileRenderMode::TILE;
    } else if(visibleTileRenderMode == VisibleTileRenderMode::TILE) {
        visibleTileRenderMode = VisibleTileRenderMode::TILE_COORDINATES;
    }
}

void Renderer::loadAndRenderCurrentVisibleTiles() {

    for(TileCords vcord : currentVisibleTiles) {
        if(loadTilesThreadsAmount > maxLoadTileThreads) {
            toShowTilesQueue.push_back(vcord);
            continue;
        }

        std::thread([vcord, this]() {
            loadAndRender(vcord);

            while(toShowTilesQueue.size() > 0 && loadTilesThreadsAmount < maxLoadTileThreads) {
                TileCords tileCords = toShowTilesQueue.back();
                toShowTilesQueue.pop_back();

                bool found = false;
                for(TileCords cv : currentVisibleTiles) {
                    found = cv.tileX == tileCords.tileX && cv.tileY == tileCords.tileY && cv.tileZ == tileCords.tileZ;
                    if(found)
                        break;
                }

                if(found) {
                    LOGI("Load and render thread. Load freshest tile.");
                    std::thread([tileCords, this]() {
                        loadAndRender(tileCords);
                    }).detach();
                }
            }
        }).detach();
    }
}


void Renderer::loadAndRender(TileCords rtc) {
    loadTilesThreadsAmount++;
    LOGI("Load and render thread started. Current threads amount: %d", loadTilesThreadsAmount);

    auto start = std::chrono::high_resolution_clock::now();

    float mapScaleFactorForCurrentVisibleTilesZ = evaluateScaleFactorFormulaForZ(rtc.tileZ);
    auto tileGeometry = tilesStorage.getTile(rtc.tileZ, rtc.tileX, rtc.tileY);
    // Этот тайл был загружен и его необходимо показать на карте
    auto needToInsertTile = TileForRenderer(
                        tileGeometry, rtc.tileXShift, rtc.tileYShift,
                            rtc.tileX, rtc.tileY, rtc.tileZ,
                            mapScaleFactorForCurrentVisibleTilesZ
                        );

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    LOGI("load tile time: %f, cords z:%d x:%d y:%d", duration.count(), rtc.tileZ, rtc.tileX, rtc.tileY);

    // Если больше нуля то тайл (needToInsertTile) выше в 3д пространстве
    // Если меньше нуля то тайл ниже
    evaluateTilesPositionsMutex.lock();

    if(needToInsertTile.tileZ != currentMapZTile())
        return;


//    bool inserted = false;
//    for(short i = 0; i < rendererTilesSize; ++i) {
//        auto& tileForR = tilesForRenderer[i];
//        if(tileForR.isEmpty() && !inserted) {
//            tileForR = needToInsertTile;
//            inserted = true;
//        }
//
//        // отображаться в первую очередь будут те тайлы
//        // что в массиве tilesForRenderer на первом месте
//        needToInsertTile.zDeltaFlag = currentMapZTile() - needToInsertTile.tileZ;
//        tileForR.zDeltaFlag = currentMapZTile() - tileForR.tileZ;
//
//        // если актуальный большой тайл закрывает маленький неактульный
//        if(needToInsertTile.zDeltaFlag == 0 && tileForR.zDeltaFlag != 0 && needToInsertTile.cover(tileForR)) {
//            tileForR.clear();
//            if(!inserted) {
//                tilesForRenderer[i] = needToInsertTile;
//                inserted = true;
//            }
//            continue;
//        }
//
//        // рисуем первым все то что ближе к текущему зуму
//        if(abs(needToInsertTile.zDeltaFlag) <= abs(tileForR.zDeltaFlag) && !inserted) {
//            for(short i1 = rendererTilesSize - 2; i1 >= i; --i1) {
//                tilesForRenderer[i1 + 1] = tilesForRenderer[i1];
//            }
//            tilesForRenderer[i] = needToInsertTile;
//            inserted = true;
//        }
//    }

    bool exist = false;
    for(short renderTileIndex = 0; renderTileIndex < rendererTilesSize; renderTileIndex++) {
        auto& tileForR = tilesForRenderer[renderTileIndex];
        if(tileForR.isEmpty())
            continue;
        exist = tileForR.tileX == needToInsertTile.tileX && tileForR.tileY == needToInsertTile.tileY && tileForR.tileZ == needToInsertTile.tileZ;
        if(exist)
            return;
    }

    short insertedIndex = -1;
    for(short renderTileIndex = 0; renderTileIndex < rendererTilesSize; renderTileIndex++) {
        auto& tileForR = tilesForRenderer[renderTileIndex];
        if(tileForR.isEmpty() || tileForR.tileZ != currentMapZTile()) {
            tilesForRenderer[renderTileIndex] = needToInsertTile;
            insertedIndex = renderTileIndex;
            break;
        }
    }

    // считаем координаты на сфере этих тайлов
    auto& tileForR = tilesForRenderer[insertedIndex];
    auto& cords = cord_tiles[insertedIndex];
    std::vector<float>().swap(cords);
    int tileZ = tileForR.tileZ;
    int n = pow(2, tileZ);
    int maxTileCord = n - 1;
    int tileX = tileForR.tileX;
    int tileY = maxTileCord - tileForR.tileY;

    for(short unitSquareIndex = 0; unitSquareIndex < sphere.unitSquareCoordinates.size(); unitSquareIndex += 2) {
        float t = sphere.unitSquareCoordinates[unitSquareIndex];
        float s = sphere.unitSquareCoordinates[unitSquareIndex + 1];
        if(t == 1 || s == 1) {
            int i = 0;
        }
        float tileXOfUnit = std::max(t * n, 0.0f);
        float tileYOfUnit = std::max(s * n, 0.0f);
        int tileXOfUnitCeil = static_cast<int>(tileXOfUnit);
        int tileYOfUnitCeil = static_cast<int>(tileYOfUnit);
        float intPart;
        float tileFractionalX = std::modf(tileXOfUnit, &intPart);
        float tileFractionalY = std::modf(tileYOfUnit, &intPart);
        if(tileXOfUnitCeil == n) {
            tileXOfUnitCeil--;
            tileFractionalX = 1.0f;
        }
        if(tileYOfUnitCeil == n) {
            tileYOfUnitCeil--;
            tileFractionalY = 1.0f;
        }
        if(tileXOfUnitCeil - 1 == tileX && tileFractionalX == 0) {
            tileXOfUnitCeil--;
            tileFractionalX = 1.0f;
        }
        if(tileYOfUnitCeil - 1 == tileY && tileFractionalY == 0) {
            tileYOfUnitCeil--;
            tileFractionalY = 1.0f;
        }
        if(tileXOfUnitCeil != tileX || tileYOfUnitCeil != tileY) {
            cords.push_back(-1.0f);
            cords.push_back(-1.0f);
            continue;
        }

        cords.push_back(tileFractionalX);
        cords.push_back(tileFractionalY);
    }

    auto& cords2 = cord_tiles[insertedIndex];
    cords2.clear();



    evaluateTilesPositionsMutex.unlock();
    loadTilesThreadsAmount--;
    LOGI("Load and render thread finished. Current threads amount: %d", loadTilesThreadsAmount);
}

TileCords Renderer::evaluateTileCords(int dX, int dY) {
    int tileXShift = *rootTileX + dX;
    int tileX_n = normalizeCoordinate(tileXShift);

    int tileYShift = *rootTileY + dY;
    int tileY_n = normalizeCoordinate(tileYShift);

    int tileLastIndex = pow(2, currentMapZTile()) - 1;
    bool xNormalized = tileXShift > tileLastIndex || tileXShift < 0;
    bool yNormalized = tileYShift > tileLastIndex || tileYShift < 0;

    //LOGI("Use mapZTileCordCurrent (evaluateTileCords) %d", mapZTile);
    //LOGI("tile dx dy %hd %hd, x y %hd %hd, mapZTileCordCurrent %hd", dX, dY, tileX_n, tileY_n, mapZTile);
    //renderTiles(mapZTile, tileX, tileY, tileX_n, tileY_n);
    return TileCords {
              tileXShift,
              tileYShift,
        tileX_n,
        tileY_n,
        currentMapZTile(),
        xNormalized || yNormalized
    };
}

void Renderer::loadAssets(AAssetManager *assetManager) {
    if(assetManager == nullptr)
        return;
    symbols->loadFont(assetManager);
}

void Renderer::updateFrustum() {
    float currentCameraZBorder = -1 * evaluateCameraZByZoomingBorder();

    // это дистанция при следующем зуме +1 к z
    float currentAndNextDistanceCamDiff = (currentCameraZBorder / 2);

    projectionMatrix = setFrustum(fovy, (float) screenW / (float) screenH,
                                  evaluateZeroZoomCameraDist() - planetRadius - currentAndNextDistanceCamDiff,
                                  evaluateZeroZoomCameraDist()
    );

    updateRenderTileProjection(2, 2);
}

void Renderer::loadTextures(AAssetManager *assetManager) {
    glGenTextures(1, &testTextureId);
    glBindTexture(GL_TEXTURE_2D, testTextureId);

    AAsset* asset = AAssetManager_open(assetManager, "images/earth2048.bmp", AASSET_MODE_UNKNOWN);
    off_t fileSize = AAsset_getLength(asset);
    unsigned char* imageData = (unsigned char*) malloc(fileSize);
    AAsset_read(asset, imageData, fileSize);
    AAsset_close(asset);

    glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            2048,
            1024,
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            imageData
    );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    free(imageData);
}

void Renderer::onSurfaceCreated(AAssetManager *assetManager) {
    getSymbols()->createFontTextures();
    loadTextures(assetManager);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_STENCIL_TEST);

//    glGenBuffers(1, &sphereVBO);
//    glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);
//    glBufferData(GL_ARRAY_BUFFER, sphere.sphere_vertices.size() * sizeof(float), sphere.sphere_vertices.data(), GL_STATIC_DRAW);
//    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenFramebuffers(1, &frameBuffer);
    glGenRenderbuffers(1, &depthBuffer);
    glGenTextures(rendererTilesSize, renderTexture);
}

void Renderer::updateRenderTileProjection(int amountX, int amountY) {
    rendererTileProjectionMatrix = setOrthoFrustum(0, 4096 * amountX, -4096 * amountY, 0, 0.1, 100);
}




