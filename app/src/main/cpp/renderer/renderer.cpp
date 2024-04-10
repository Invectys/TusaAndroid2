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
#include <cmath>
#include <iostream>
#include <thread>

bool DEBUG_TILE = true;
bool UPDATE_ON_DRAG = false;

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
    auto start = std::chrono::high_resolution_clock::now();
    glStencilMask(0xFF);
    glClearColor((float)242 / 255, (float)248 / 255, (float)230 / 255, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);


    Matrix4 viewMatrix = Matrix4();
    viewMatrix.translate(camera[0], camera[1], camera[2]);
    Matrix4 pvm = projectionMatrix * viewMatrix;

    for(int i = 0; i < rendererTilesSize; ++i) {
        TileForRenderer visibleTile = tilesForRenderer[i];
        if(visibleTile.isEmpty())
            continue;

        float renderTileScaleFactor = visibleTile.mapScaleFactor;
        Matrix4 modelMatrix = Matrix4();

        float tileExtent = evaluateExtentForScaleFactor(renderTileScaleFactor);
        modelMatrix.scale(renderTileScaleFactor, renderTileScaleFactor, 1);
        modelMatrix.translate(visibleTile.shiftX * tileExtent, -1 * visibleTile.shiftY * tileExtent, 0);

        if(visibleTileRenderMode == VisibleTileRenderMode::TILE) {
            // Рисуем сам тайл, геометрию
            renderTileGeometry.render(pvm, modelMatrix, visibleTile.tile);
        } else if(visibleTileRenderMode == VisibleTileRenderMode::TILE_COORDINATES) {
            Matrix4 pvmm = pvm * modelMatrix;
            renderTileCoordinates->render(pvmm, visibleTile, extent);
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    //LOGI("render time: %f", duration.count());
}

void Renderer::onSurfaceChanged(int w, int h) {
    screenW = w;
    screenH = h;
    updateFrustum();
}

void Renderer::updateVisibleTiles(bool render) {
    if(DEBUG_TILE) {
        currentVisibleTiles = {
                TileCords {0, 0, 0, 0, 1},
                TileCords {0, 1, 0, 1, 1},
                TileCords {0, 0, 0, 0, 0} ,
               //TileCords {1, 0, 1, 0, 1} ,


               //TileCords {1, 1, 1, 1, 1} ,
        };
        loadAndRenderCurrentVisibleTiles();
        return;
    }

    std::vector<TileCords> newVisibleTiles = {};
    auto topCords = evaluateTileCords(0, 0);
    auto bottomCords = evaluateTileCords(0, 1);
    newVisibleTiles.push_back(topCords);
    newVisibleTiles.push_back(bottomCords);
//    std::thread([this]() {
//    }).detach();

    RenderTilesEnum renderTilesEnum = evaluateRenderTilesCameraXType();
    if(renderTilesEnum == RenderTilesEnum::SHOW_RIGHT) {
        LOGI("Show tiles direction: SHOW_RIGHT");
        auto topRightCords = evaluateTileCords(1, 0);
        auto bottomRightCords = evaluateTileCords(1, 1);

        newVisibleTiles.push_back(topRightCords);
        newVisibleTiles.push_back(bottomRightCords);
    } else if(renderTilesEnum == RenderTilesEnum::SHOW_LEFT) {
        LOGI("Show tiles direction: SHOW_LEFT");
        auto topLeftCords = evaluateTileCords(-1, 0);
        auto bottomLeftCords = evaluateTileCords(-1, 1);

        newVisibleTiles.push_back(topLeftCords);
        newVisibleTiles.push_back(bottomLeftCords);
    }

    currentVisibleTiles = std::move(newVisibleTiles);
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
    camera[0] -= dx * dragFingerMapSpeed * scaleForDrag;
    camera[1] += dy * dragFingerMapSpeed * scaleForDrag;
    if (!UPDATE_ON_DRAG)
        return;

    auto rootTileXEv = evaluateRootTileX();
    auto rootTileYEv = evaluateRooTileY();

    bool needUpdateVisibleTiles = false;
    if(updateSavedLastDragXTileType()) {
        needUpdateVisibleTiles = true;
    }

    if(rootTileXEv != *rootTileX || rootTileYEv != *rootTileY) {
        updateRootTile();
        needUpdateVisibleTiles = true;
    }

    if(needUpdateVisibleTiles) {
        updateVisibleTiles();
    }
}

void Renderer::scale(float factor) {
    updateMapZoomScaleFactor(factor);
    LOGI("Scale factor %f", scaleFactorZoom);

    updateMapCameraZPosition();
    if(_savedLastScaleStateMapZ != currentMapZTile()) {
        renderTileGeometry.scaleZCordDrawHeapsDiff(evaluateScaleFactorFormula());
        updateFrustum();
        updateRootTile();
        updateSavedLastDragXTileType();
        updateVisibleTiles();
        _savedLastScaleStateMapZ = currentMapZTile();
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
    for(TileCords& vcord : currentVisibleTiles) {
        loadAndRender(vcord);
    }
}

void Renderer::loadAndRender(TileCords renderTileCords) {
    short currentTileZ = currentMapZTile();
    float mapScaleFactorForCurrentVisibleTilesZ = evaluateScaleFactorFormulaForZ(renderTileCords.tileZ);
    auto tileGeometry = tilesStorage.getTile(renderTileCords.tileZ, renderTileCords.tileX, renderTileCords.tileY);
    auto needToInsertTile = TileForRenderer(
                        tileGeometry, renderTileCords.tileXShift, renderTileCords.tileYShift,
                            renderTileCords.tileX, renderTileCords.tileY, renderTileCords.tileZ,
                            mapScaleFactorForCurrentVisibleTilesZ
                        );

    // Если больше нуля то тайл (needToInsertTile) выше в 3д пространстве
    // Если меньше нуля то тайл ниже
    needToInsertTile.zDeltaFlag = currentTileZ - needToInsertTile.tileZ;

    bool inserted = false;


    for(short i = 0; i < rendererTilesSize; ++i) {
        auto& tileForR = tilesForRenderer[i];
        if(tileForR.isEmpty() && !inserted) {
            tileForR = needToInsertTile;
            inserted = true;
        }
        tileForR.zDeltaFlag = currentTileZ - tileForR.tileZ;

        if(needToInsertTile.zDeltaFlag < tileForR.zDeltaFlag && !inserted) {
            for(short i1 = rendererTilesSize - 2; i1 >= i; --i1) {
                tilesForRenderer[i1 + 1] = tilesForRenderer[i1];
            }
            tilesForRenderer[i] = needToInsertTile;
            inserted = true;
        }

        if(needToInsertTile.zDeltaFlag > tileForR.zDeltaFlag && tileForR.zDeltaFlag != 0) {
            if(needToInsertTile.cover(tileForR)) {
                tileForR.clear();
                if(!inserted) {
                    tileForR = needToInsertTile;
                    inserted = true;
                }
            }
        }
    }
}

TileCords Renderer::evaluateTileCords(int dX, int dY) {
    int tileXShift = *rootTileX + dX;
    int tileX_n = normalizeCoordinate(tileXShift);

    int tileYShift = *rootTileY + dY;
    int tileY_n = normalizeCoordinate(tileYShift);

    //LOGI("Use mapZTileCordCurrent (evaluateTileCords) %d", mapZTile);
    //LOGI("tile dx dy %hd %hd, x y %hd %hd, mapZTileCordCurrent %hd", dX, dY, tileX_n, tileY_n, mapZTile);
    //renderTiles(mapZTile, tileX, tileY, tileX_n, tileY_n);
    return TileCords {
              tileXShift,
              tileYShift,
        tileX_n,
        tileY_n,
        currentMapZTile()
    };
}

void Renderer::loadAssets(AAssetManager *assetManager) {
    if(assetManager == nullptr)
        return;
    symbols->loadFont(assetManager);
}

void Renderer::updateFrustum() {
    float currentCameraZBorder = -1 * evaluateCameraZByZoomingBorder();
    float maxZDistance = renderTileGeometry.getZCordDrawHeapDiff() * Style::maxGeometryHeaps;

    // это дистанция при следующем зуме +1 к z
    float currentAndNextDistanceCamDiff = (currentCameraZBorder / 2);

    projectionMatrix = setFrustum(fovy, (float) screenW / (float) screenH,
                                  currentCameraZBorder - maxZDistance - currentAndNextDistanceCamDiff,
                                  currentCameraZBorder + 1
                                  );
}


