//
// Created by Artem on 02.01.2024.
//

#ifndef TUSA_RENDERER_H
#define TUSA_RENDERER_H

#include "shader/shaders_bucket.h"
#include "map/tile.h"
#include "map/tiles_storage.h"
#include "style/style.h"
#include "map/visible_tile.h"
#include "render_tile_geometry.h"
#include "visible_tile_render_mode.h"
#include "symbols/symbols.h"
#include "render_tile_coordinates.h"
#include <vector>


class Renderer {
public:
    Renderer(std::shared_ptr<ShadersBucket> shadersBucket, Cache* cache);
    ~Renderer() {
        delete topLeftTileXTilesView;
        delete topLeftTileYTilesView;
    }

    void onSurfaceChanged(int w, int h);
    void renderFrame();
    void updateVisibleTiles();
    void drag(float dx, float dy);
    void scale(float scaleFactor);
    void doubleTap();
    int normalizeCoordinate(int coordinate);

    // Устанавливает камеру в центр тайла
    void updateCordsByTiles(int xTile, int yTile) {
        *topLeftTileXTilesView = xTile;
        *topLeftTileYTilesView = yTile;
        auto _currentExtent = currentExtent();
        cameraXStart = -(float) _currentExtent / 2.0 - *topLeftTileXTilesView * _currentExtent;
        cameraYStart =  (float) _currentExtent / 2.0 + *topLeftTileYTilesView * _currentExtent;
        camera[0] = cameraXStart;
        camera[1] = cameraYStart;
    }

    void updateTopLeftXYTileShowed() {
        auto _currentExtent = currentExtent();
        float cameraXCord = camera[0];
        float cameraYCord = camera[1];
        int centerX = -1 * cameraXCord / _currentExtent;
        int centerY = cameraYCord / _currentExtent;
        *topLeftTileXTilesView = centerX;
        *topLeftTileYTilesView = centerY;
    }

    void setupNoOpenGLMapState(float scaleFactor, AAssetManager *assetManager) {
        topLeftTileXTilesView = new int();
        topLeftTileYTilesView = new int();
        loadAssets(assetManager);
        cameraRootZ = -5000;
        fovy = 80;
        updateMapZoomScaleFactor(scaleFactor);
        updateCordsByTiles(0, 0);
        updateMapCameraZPosition();
        _savedLastScaleStateMapZ = currentMapZTile();
        renderTileGeometry.scaleZCordDrawHeapsDiff(evaluateScaleFactorFormula());
        updateVisibleTiles();
    }

    std::shared_ptr<Symbols> getSymbols() {return symbols;}

    VisibleTileRenderMode visibleTileRenderMode = VisibleTileRenderMode::TILE_COORDINATES;

    void loadAssets(AAssetManager *assetManager);
    void showTile(int dX, int dY, short index);
    void showTile(short zoom, int shiftX, int shiftY, int x, int y, short index);
private:
    short mapZTileCordMax = 19;
    float mapZTileScalingRoot = 12;
    float scaleFactorZoom = 0.0f; // из MapView устанавливается

    float _savedLastScaleStateMapZ;

    void updateMapZoomScaleFactor(float scaleFactor) {
        scaleFactorZoom = scaleFactor - 1.0f;
    }

    void updateMapCameraZPosition() {
        camera[2] = cameraRootZ * evaluateFloatScaleFactorFormula();
    }

    float evaluateCameraZByZoomingBorder(short zoomDiff = 0) {
        return cameraRootZ * evaluateScaleFactorFormula(zoomDiff);
    }

    void updateFrustum();

    float evaluateFloatScaleFactorFormula() {
        return pow(2, mapZTileScalingRoot - scaleFactorZoom);
    }

    float evaluateScaleFactorFormulaForZ(short z, short zoomDiff = 0) {
        return pow(2, mapZTileScalingRoot - z - zoomDiff);
    }

    float evaluateScaleFactorFormula(short zoomDiff = 0) {
        return evaluateScaleFactorFormulaForZ(currentMapZTile(), zoomDiff);
    }

    short currentMapZTile() {
        return (short) scaleFactorZoom;
    }

    float evaluateExtentForScaleFactor(float scaleFactor) {
        return (float) extent * scaleFactor;
    }

    float currentExtent() {
        return evaluateExtentForScaleFactor(evaluateScaleFactorFormula());
    }

    std::shared_ptr<ShadersBucket> shadersBucket;
    RenderTileGeometry renderTileGeometry;
    std::shared_ptr<RenderTileCoordinates> renderTileCoordinates;
    std::shared_ptr<Symbols> symbols;
    int* topLeftTileXTilesView, *topLeftTileYTilesView;

    Matrix4 projectionMatrix;

    const uint32_t extent = 4096;
    float dragFingerMapSpeed = 2.0f;

    float fovy;
    float cameraXStart;
    float cameraYStart;
    float cameraRootZ;
    float camera[3] = {};
    int screenW, screenH;
    Cache* cache;
    TilesStorage tilesStorage = TilesStorage(cache);
    VisibleTile visibleTiles[9] = {};
};


#endif //TUSA_RENDERER_H
