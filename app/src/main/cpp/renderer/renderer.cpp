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
#include "util/eigen_gl.h"
#include <cmath>
#include <iostream>
#include <thread>
#include <stack>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>

bool DEBUG_TILE = false;
bool UPDATE_ON_DRAG = true;
bool DEBUG = false;


std::mutex evaluateTilesPositionsMutex;
std::mutex regeneratePlanetGeometryMutex;

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

void Renderer::renderFrame() {

    //evaluateTilesPositionsMutex.lock();
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<PlainShader> plainShader = shadersBucket->plainShader;
    std::shared_ptr<SymbolShader> symbolShader = shadersBucket->symbolShader;
    std::shared_ptr<PlanetShader> planetShader = shadersBucket->planetShader;

    std::set<short> possibleDeltasSet = {};
    std::vector<short> possibleDeltasVec = {};
    for(short renderTile = 0; renderTile < tilesForRenderMaxSize; renderTile++) {
        TileForRenderer visibleTile = tilesForRenderer[0];
        if(visibleTile.isEmpty())
            continue;
        if(possibleDeltasSet.insert(visibleTile.zDeltaFlag).second) {
            possibleDeltasVec.push_back(visibleTile.zDeltaFlag);
        }
    }

    Eigen::Matrix4f viewMatrixTexture = Eigen::Matrix4f::Identity();
    Eigen::Affine3f viewTranslation(Eigen::Translation3f(0, 0, -1));
    viewMatrixTexture *= viewTranslation.matrix();
    Eigen::Matrix4f pvmTexture = rendererTileProjectionMatrix * viewMatrixTexture;

    if (!RENDER_TILE_PALLET_TEST) {
        glBindTexture(GL_TEXTURE_2D, renderMapTexture);
//        float mapTextureResolutionScale = 1;
//        float resultMapTextureWidth = mapTextureResolutionScale * renderMapTextureWidth;
//        float resultMapTextureHeight = mapTextureResolutionScale * renderMapTextureHeight;
//        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, resultMapTextureWidth , resultMapTextureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, renderMapFrameBuffer);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderMapTexture, 0);
        //glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, resultMapTextureWidth, resultMapTextureHeight);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderMapDepthBuffer);
        glViewport(0, 0, renderMapTextureWidth, renderMapTextureHeight);
    }

    glStencilMask(0xFF);
    glClearColor((float)242 / 255, (float)248 / 255, (float)230 / 255, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glStencilFunc(GL_GREATER, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glUseProgram(plainShader->program);

    auto vt = currentVisibleTiles;
    auto renderTileCoordinatesPtr = renderTileCoordinates.get();
    if(visibleTileRenderMode == VisibleTileRenderMode::TILE) {
        short reserved = 1;
        for(short zDelta : possibleDeltasVec) {
            for(short geometryHeapIndex = 0; geometryHeapIndex < Style::maxGeometryHeaps + reserved; ++geometryHeapIndex) {
                for(short renderTileIndex = 0; renderTileIndex < tilesForRenderMaxSize; renderTileIndex++) {
                    TileForRenderer visibleTile = tilesForRenderer[renderTileIndex];
                    if(visibleTile.isEmpty() || visibleTile.zDeltaFlag != zDelta)
                        continue;

                    short deltaX = visibleTile.rPosX;
                    short deltaY = visibleTile.rPosY;

                    short shift = extent;
                    Eigen::Affine3f modelTranslation(Eigen::Translation3f(shift * deltaX, -1 * shift * deltaY, 0));
                    Eigen::Matrix4f forTileMatrix = pvmTexture * modelTranslation.matrix();
                    glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, forTileMatrix.data());

                    Tile* tile = visibleTile.tile;
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
    }

    glViewPortDefaultSize();

    if (!RENDER_TILE_PALLET_TEST) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glStencilMask(0x0);

        drawPlanet();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    //LOGI("render time: %f", duration.count());w
    //evaluateTilesPositionsMutex.unlock();
}



void Renderer::drawPlanet() {
    // 1_drawPlanet
    regeneratePlanetGeometryMutex.lock();
    std::shared_ptr<PlanetShader> planetShader = shadersBucket->planetShader;
    std::shared_ptr<PlainShader> plainShader = shadersBucket->plainShader;
    glUseProgram(planetShader->program);

    // отрендрена область размера
    int n = pow(2, topLeftVisibleCord.tileZ);
    float tileP = 1.0 / n;

    float xStartBorder = topLeftVisibleCord.tileX * tileP;
    float xEndBorder = tileP * renderMapXTilesCount + xStartBorder;
    float yStartBorder = topLeftVisibleCord.tileY * tileP;
    float yEndBorder = tileP * renderMapYTilesCount + yStartBorder;

    glUniform1f(planetShader->getStartX(), xStartBorder);
    glUniform1f(planetShader->getEndX(),   xEndBorder);
    glUniform1f(planetShader->getStartY(), yStartBorder);
    glUniform1f(planetShader->getEndY(),   yEndBorder);

    Eigen::Matrix4f pvm = evaluatePVM();
    glUniformMatrix4fv(planetShader->getMatrixLocation(), 1, GL_FALSE, pvm.data());

    glVertexAttribPointer(planetShader->getUnitSquareCoordinates(), 2, GL_FLOAT, GL_FALSE, 0, sphere.unitSquareCoordinates.data());
    glEnableVertexAttribArray(planetShader->getUnitSquareCoordinates());
    glVertexAttribPointer(planetShader->getPosLocation(), 3, GL_FLOAT, GL_FALSE, 0, sphere.sphere_vertices.data());
    glEnableVertexAttribArray(planetShader->getPosLocation());

    glBindTexture(GL_TEXTURE_2D, renderMapTexture);
    glUniform1i(planetShader->getTileTextureLocation0(), 0);
    glDrawElements(GL_TRIANGLES, sphere.sphere_indices.size(), GL_UNSIGNED_INT, sphere.sphere_indices.data());

    //drawPoints(pvm, sphere.sphere_vertices, 7.0f);
    drawPoint(pvm, 0, 0, 0, 10.0f);

    regeneratePlanetGeometryMutex.unlock();
}


void Renderer::onSurfaceChanged(int w, int h) {
    screenW = w;
    screenH = h;
    glViewPortDefaultSize();
    updateFrustum();
    updateVisibleTiles();

    glBindTexture(GL_TEXTURE_2D, renderMapTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, renderMapTextureWidth, renderMapTextureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindRenderbuffer(GL_RENDERBUFFER, renderMapDepthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, renderMapTextureWidth, renderMapTextureHeight);
}

void Renderer::updateVisibleTiles() {
    // 1_updateVisibleTiles

    if (DEBUG) {
        return;
    }
    Eigen::Matrix4f pvm = evaluatePVM();
    auto corners = evaluateCorners(pvm);
    short z = currentMapZTile();
    float n = pow(2, z);

    float leftTop_x = 0;
    float leftTop_y = 0;
    float leftTop_xTile = 0;
    float leftTop_yTile = 0;
    if (corners.hasLeftTop) {
        leftTop_x = (corners.leftTopLongitudeRad + M_PI) / (2 * M_PI);
        leftTop_y = CommonUtils::latitudeRadToY(corners.leftTopLatitudeRad);
        leftTop_xTile = leftTop_x * n;
        leftTop_yTile = leftTop_y * n;
    }

    float rightBottom_x = 1;
    float rightBottom_y = 1;
    float rightBottom_xTile = n - 1;
    float rightBottom_yTile = n - 1;
    if (corners.hasRightBottom) {
        rightBottom_x = (CommonUtils::normalizeLongitudeRad(corners.rightBottomLongitudeRad) + M_PI) / (2 * M_PI);
        rightBottom_y = CommonUtils::latitudeRadToY(corners.rightBottomLatitudeRad);
        rightBottom_xTile = rightBottom_x * n;
        rightBottom_yTile = rightBottom_y * n;
    }

    std::vector<TileCords> newVisibleTiles = {};
    for (int xTile = (int)leftTop_xTile; xTile <= (int)rightBottom_xTile; xTile++) {
        for (int yTile = (int)leftTop_yTile; yTile <= (int)rightBottom_yTile; yTile++) {
            auto tileCord = getClearTileCord(xTile, yTile, z, xTile - (int) leftTop_xTile,yTile - (int) leftTop_yTile);
            newVisibleTiles.push_back(tileCord);
        }
    }

    int renderXDiffSize = (int) rightBottom_xTile - (int) leftTop_xTile + 1;
    int renderYDiffSize = (int) rightBottom_yTile - (int) leftTop_yTile + 1;
    updateRenderTileProjection(renderXDiffSize, renderYDiffSize);
    topLeftVisibleCord = getClearTileCord(leftTop_xTile, leftTop_yTile, z, 0 ,0);
    topLeftVisibleCord.xComponent = leftTop_x;
    topLeftVisibleCord.yComponent = leftTop_y;
    topLeftVisibleCord.longitudeRad = corners.leftTopLongitudeRad;
    topLeftVisibleCord.latitudeRad = corners.leftTopLatitudeRad;
    topLeftVisibleCord.hasLatitudeAndLongitudeRad = corners.hasLeftTop;

    bottomRightVisibleCord = getClearTileCord(rightBottom_xTile, rightBottom_yTile, z, 0 ,0);
    bottomRightVisibleCord.xComponent = rightBottom_x;
    bottomRightVisibleCord.yComponent = rightBottom_y;
    bottomRightVisibleCord.longitudeRad = corners.rightBottomLongitudeRad;
    bottomRightVisibleCord.latitudeRad = corners.rightBottomLatitudeRad;
    bottomRightVisibleCord.hasLatitudeAndLongitudeRad = corners.hasRightBottom;

    // end_updateVisibleTiles

    std::ostringstream oss;
    for (auto& tile : newVisibleTiles) {
        oss << tile.toString() << " ";
    }
    LOGI("[SHOW_PIPE] New visible tiles: %s", oss.str().c_str());

    currentVisibleTiles = std::move(newVisibleTiles);
    toShowTilesQueue.clear();
    loadAndRenderCurrentVisibleTiles();
}


void Renderer::drag(float dx, float dy) {
    float scaleForDrag = evaluateFloatScaleFactorFormula();
    cameraLatitudeRad += dy * dragFingerMapSpeed * scaleForDrag;
    cameraLatitudeRad = std::max(-latitudeCameraAngleRadConstraint, std::min(cameraLatitudeRad, latitudeCameraAngleRadConstraint));
    cameraLongitudeRad += dx * dragFingerMapSpeed * scaleForDrag;
    LOGI("Planet coordinates: Latitude: %f Longitude %f", getPlanetCurrentLatitude(), getPlanetCurrentLongitude());

    if (!UPDATE_ON_DRAG)
        return;

    auto rootTileXEv = evaluateRootTileDiffX();
    auto rootTileYEv = evaluateRootTileDiffY();

    updatePlanetGeometry();
    bool needUpdateVisibleTiles = false;

//    if(needUpdateVisibleTiles) {
//        updateVisibleTiles();
//    }
}

void Renderer::scale(float factor) {
    LOGI("Factor %f", factor);
    updateMapZoomScaleFactor(factor); // обновляет текущий скейл, паарметр скейла
    LOGI("Scale factor %f", scaleFactorZoom);

    updateCameraPosition(); // дистанция камеры от центра
    if(_savedLastScaleStateMapZ != currentMapZTile()) {
        // Новый зум у карты. Новая координата z
        renderTileGeometry.scaleZCordDrawHeapsDiff(evaluateScaleFactorFormula());

        updateFrustum();
        updateVisibleTiles();
        _savedLastScaleStateMapZ = currentMapZTile();
        LOGI("New Z of map %d", currentMapZTile());
        LOGI("Current tile extent %f", evaluateCurrentExtent());
    }
}

void Renderer::doubleTap() {
    RENDER_TILE_PALLET_TEST = !RENDER_TILE_PALLET_TEST;
    //DEBUG = !DEBUG;
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
                            mapScaleFactorForCurrentVisibleTilesZ,
                            rtc.renderPosX, rtc.renderPosY,
                            rtc.xNormalized, rtc.yNormalized
                        );

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    LOGI("load tile time: %f, cords z:%d x:%d y:%d", duration.count(), rtc.tileZ, rtc.tileX, rtc.tileY);

    // Если больше нуля то тайл (needToInsertTile) выше в 3д пространстве
    // Если меньше нуля то тайл ниже
    //evaluateTilesPositionsMutex.lock();

    if(needToInsertTile.tileZ != currentMapZTile()) {
        loadTilesThreadsAmount--;
        return;
    }

    // удалить все то что не того Z
    for(short renderTileIndex = 0; renderTileIndex < tilesForRenderMaxSize; renderTileIndex++) {
        auto& tileForR = tilesForRenderer[renderTileIndex];
        if(tileForR.tileZ != currentMapZTile()) {
            tilesForRenderer[renderTileIndex].clear();
        }
    }

    // тайл уже добавлен
    bool exist = false;
    for(short renderTileIndex = 0; renderTileIndex < tilesForRenderMaxSize; renderTileIndex++) {
        auto& tileForR = tilesForRenderer[renderTileIndex];
        if(tileForR.isEmpty())
            continue;
        exist = tileForR.tileX == needToInsertTile.tileX && tileForR.tileY == needToInsertTile.tileY && tileForR.tileZ == needToInsertTile.tileZ;
        if(exist)
            break;
    }

    if (!exist) {
        short insertedIndex = -1;
        for(short renderTileIndex = 0; renderTileIndex < tilesForRenderMaxSize; renderTileIndex++) {
            auto& tileForR = tilesForRenderer[renderTileIndex];
            if(tileForR.isEmpty() || tileForR.tileZ != currentMapZTile()) {
                tilesForRenderer[renderTileIndex] = needToInsertTile;
                insertedIndex = renderTileIndex;
                break;
            }
        }
        LOGI("[SHOW_PIPE] loadAndRender() %s ins idx = %d", needToInsertTile.toString().c_str(), insertedIndex);
    }

    //evaluateTilesPositionsMutex.unlock();
    loadTilesThreadsAmount--;
    LOGI("Load and render thread finished. Current threads amount: %d", loadTilesThreadsAmount);
}

void Renderer::loadAssets(AAssetManager *assetManager) {
    if(assetManager == nullptr)
        return;
    symbols->loadFont(assetManager);
}

void Renderer::updateFrustum() {
    // сколько нужно проскролить до нового зума
    float currentCameraZBorder = evaluateCameraZByZoomingBorder();

    // это дистанция при следующем зуме +1 к z
    float currentAndNextDistanceCamDiff = (currentCameraZBorder / 2);
    projectionMatrix = EigenGL::createPerspectiveProjectionMatrix(
            fovy,
            (float) screenW / (float) screenH,
            currentCameraZBorder - currentAndNextDistanceCamDiff,
            currentCameraZBorder + planetRadius
    );
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
    updatePlanetGeometry();

    glGenFramebuffers(1, &renderMapFrameBuffer);
    glGenRenderbuffers(1, &renderMapDepthBuffer);
    glGenTextures(1, &renderMapTexture);
}

void Renderer::updateRenderTileProjection(short amountX, short amountY) {
    renderMapXTilesCount = amountX;
    renderMapYTilesCount = amountY;
    rendererTileProjectionMatrix = EigenGL::createOrthoMatrix(0, 4096 * amountX, -4096 * amountY, 0, 0.1, 100);
}

CornersCords Renderer::evaluateCorners(Eigen::Matrix4f pvm) {
    std::vector<Eigen::Vector4f> planes = EigenGL::extractFrustumPlanes(pvm);
    auto leftPlane = planes[0];
    auto topPlane = planes[3];
    float topLeftLongitudeRad = 0;
    float topLeftLatitudeRad = 0;
    bool hasTopLeft = false;
    evaluateLatLonByIntersectionPlanes(leftPlane, topPlane, pvm, topLeftLongitudeRad, topLeftLatitudeRad, hasTopLeft);

    auto rightPlane = planes[1];
    auto bottomPlane = planes[2];
    float bottomRightLongitudeRad = 0;
    float bottomRightLatitudeRad = 0;
    bool hasBottomRight = false;
    evaluateLatLonByIntersectionPlanes(rightPlane, bottomPlane, pvm, bottomRightLongitudeRad, bottomRightLatitudeRad, hasBottomRight);

    return CornersCords {
            topLeftLongitudeRad,
            topLeftLatitudeRad,
            bottomRightLongitudeRad,
            bottomRightLatitudeRad,
            hasTopLeft,
            hasBottomRight
    };
}

void Renderer::evaluateLatLonByIntersectionPlanes(
        Eigen::Vector4f firstPlane, Eigen::Vector4f secondPlane,
        Eigen::Matrix4f pvm, float& longitudeRad, float& latitudeRad, bool& has) {
    float A1 = firstPlane[0];
    float B1 = firstPlane[1];
    float C1 = firstPlane[2];
    float D1 = firstPlane[3];

    float A2 = secondPlane[0];
    float B2 = secondPlane[1];
    float C2 = secondPlane[2];
    float D2 = secondPlane[3];

    float leftTopPlaneLineVector[3] = {
            B1 * C2 - C1 * B2, // a
            C1 * A2 - A1 * C2, // b
            A1 * B2 - B1 * A2 // c
    };

    float det = A1 * B2 - A2 * B1;
    float detX = (-D1) * B2 - (-D2) * B1;
    float detY = A1 * (-D2) - A2 * (-D1);

    float x0 = detX / det;
    float y0 = detY / det;
    float z0 = 0; // потому что мы взяли его как свободный параметр для нахождения любой точки

    float a = leftTopPlaneLineVector[0];
    float b = leftTopPlaneLineVector[1];
    float c = leftTopPlaneLineVector[2];

    float Ay = pow(a, 2.0) + pow(b, 2.0) + pow(c, 2.0);
    float By = 2 * (x0 * a + y0 * b + z0 * c);
    float Cy = pow(x0, 2.0) + pow(y0, 2.0) + pow(z0, 2.0) - pow(planetRadius, 2.0);
    float discremenant = pow(By, 2.0) - 4 * Ay * Cy;
    if (discremenant > 0) {
        float t1 = (-By + sqrt(discremenant)) / (2 * Ay);
        float t2 = (-By - sqrt(discremenant)) / (2 * Ay);
//        float xi1 = x0 + a * t1;
//        float yi1 = y0 + b * t1;
//        float zi1 = z0 + c * t1;

        // точка пересечения ближайшая к камере
        float xi2 = x0 + a * t2;
        float yi2 = y0 + b * t2;
        float zi2 = z0 + c * t2;

        longitudeRad = atan2(xi2, zi2) + M_PI / 2;
        longitudeRad = CommonUtils::normalizeLongitudeRad(longitudeRad);

        float hypotenuse = sqrt(pow(xi2, 2.0) + pow(zi2, 2.0));
        latitudeRad = -atan2(yi2, hypotenuse);
        has = true;
        return;
    }

    longitudeRad = 0;
    latitudeRad = 0;
    has = false;
}

void Renderer::updatePlanetGeometry() {
    regeneratePlanetGeometryMutex.lock();

    double longitudeDelta = M_PI;
    double latitudeDelta = M_PI / 2;
    if (topLeftVisibleCord.hasLatitudeAndLongitudeRad && bottomRightVisibleCord.hasLatitudeAndLongitudeRad) {
        longitudeDelta = abs(bottomRightVisibleCord.longitudeRad - topLeftVisibleCord.longitudeRad);
        latitudeDelta = abs(bottomRightVisibleCord.latitudeRad - topLeftVisibleCord.latitudeRad);
    }

    if (!DEBUG) {
        sphere.generateSphereData5(30, 30,
                                   planetRadius,
                                   -cameraLatitudeRad,
                                   -cameraLongitudeRad,
                                   longitudeDelta,
                                   latitudeDelta);
    }

    regeneratePlanetGeometryMutex.unlock();
}



