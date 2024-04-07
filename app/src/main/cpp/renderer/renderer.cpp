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

bool DEBUG_TILE = false;
short DEBUG_TILE_X = 0;
short DEBUG_TILE_Y = 0;
short DEBUG_TILE_ZOOM = 0;

//short DEBUG_TILE_X = 0;
//short DEBUG_TILE_Y = 0;
//short DEBUG_TILE_ZOOM = 0;

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

    glClearColor((float)242 / 255, (float)248 / 255, (float)230 / 255, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Matrix4 viewMatrix = Matrix4();
    viewMatrix.translate(camera[0], camera[1], camera[2]);
    Matrix4 pvm = projectionMatrix * viewMatrix;

    for(int i = 0; i < 9; ++i) {
        VisibleTile visibleTile = visibleTiles[i];
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

void Renderer::updateVisibleTiles() {
    if(DEBUG_TILE) {
        showTile(DEBUG_TILE_ZOOM, 0, 0, DEBUG_TILE_X, DEBUG_TILE_Y, 0);
        return;
    }

    // top-left
    std::thread([this]() {
        showTile(0, 0, 0);
    }).detach();

    // top-right
    std::thread([this]() {
        showTile(1, 0, 1);
    }).detach();

    // bottom-left
    std::thread([this]() {
        showTile(0, -1, 1);
    }).detach();

    // bottom-right
    std::thread([this]() {
        showTile(1, -1, 1);
    }).detach();


//    std::thread *loadBordersTilesThread = new std::thread([&]() {
//        showTile(-1, 1, 5);
//        showTile(1, 1, 6);
//        showTile(1, -1, 7);
//        showTile(-1, -1, 8);
//    });
    //tileThreads.push_back(loadBordersTilesThread);
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
}

void Renderer::scale(float factor) {
    updateMapZoomScaleFactor(factor);
    LOGI("Scale factor %f", scaleFactorZoom);

    updateMapCameraZPosition();
    if(_savedLastScaleStateMapZ != currentMapZTile()) {
        renderTileGeometry.scaleZCordDrawHeapsDiff(evaluateScaleFactorFormula());
        updateFrustum();
        updateCenterXYTileShowing();
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

void Renderer::showTile(short z, int shiftX, int shiftY, int x, int y, short index) {
    Tile* tile = tilesStorage.getTile(z, x, y);
    visibleTiles[index] = VisibleTile(tile, shiftX, shiftY, evaluateScaleFactorFormulaForZ(z));
}

void Renderer::showTile(int dX, int dY, short index) {
    int tileX = *topLeftTileXTilesView + dX;
    int tileX_n = normalizeCoordinate(tileX);

    int tileY = *topLeftTileYTilesView + dY;
    int tileY_n = normalizeCoordinate(tileY);

    short mapZTile = currentMapZTile();
    LOGI("Use mapZTileCordCurrent (showTile) %d", mapZTile);
    LOGI("tile dx dy %hd %hd, x y %hd %hd, mapZTileCordCurrent %hd", dX, dY, tileX_n, tileY_n, mapZTile);
    showTile(mapZTile, tileX, tileY, tileX_n, tileY_n, index);
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