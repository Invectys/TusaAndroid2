//
// Created by Artem on 02.01.2024.
//

#ifndef TUSA_RENDERER_H
#define TUSA_RENDERER_H

#include "shader/shaders_bucket.h"
#include "map/tile.h"
#include "map/tiles_storage.h"
#include "style/style.h"
#include "map/tile_for_renderer.h"
#include "render_tile_geometry.h"
#include "visible_tile_render_mode.h"
#include "symbols/symbols.h"
#include "render_tile_coordinates.h"
#include "render_tiles_enum.h"
#include "tile_cords.h"
#include <vector>


class Renderer {
public:
    Renderer(std::shared_ptr<ShadersBucket> shadersBucket, Cache* cache);
    ~Renderer() {
        delete rootTileX;
        delete rootTileY;
    }

    void onSurfaceChanged(int w, int h);
    void renderFrame();
    void updateVisibleTiles(bool render = true);
    void drag(float dx, float dy);
    void scale(float scaleFactor);
    void doubleTap();
    int normalizeCoordinate(int coordinate);

    float proportionXCamCordByCurrentExtent() {
        auto tileRemain = fmod(-camera[0], currentExtent());
        return tileRemain / currentExtent();
    }

    // Устанавливает камеру в центр тайла
    void updateCordsByTiles(int xTile, int yTile) {
        *rootTileX = xTile;
        *rootTileY = yTile;
        auto _currentExtent = currentExtent();
        auto cameraXStart = - *rootTileX * _currentExtent - _currentExtent / 2;
        auto cameraYStart = *rootTileY * _currentExtent + _currentExtent;
        camera[0] = cameraXStart;
        camera[1] = cameraYStart;
    }

    int evaluateRootTileX() {
        return -1 * camera[0] / currentExtent();
    }

    int evaluateRooTileY() {
        auto _currentExtent = currentExtent();
        return (camera[1] - _currentExtent / 2) / _currentExtent;
    }

    void updateRootTile() {
        *rootTileX = evaluateRootTileX();
        *rootTileY = evaluateRooTileY();
    }

    void setupNoOpenGLMapState(float scaleFactor, AAssetManager *assetManager) {
        showTilesXCordProportion = 0.3;
        rootTileX = new int();
        rootTileY = new int();
        loadAssets(assetManager);
        cameraRootZ = -3500;
        fovy = 65;
        updateMapZoomScaleFactor(scaleFactor);
        updateCordsByTiles(0, 0);
        updateMapCameraZPosition();
        _savedLastScaleStateMapZ = currentMapZTile();
        renderTileGeometry.scaleZCordDrawHeapsDiff(evaluateScaleFactorFormula());
        updateVisibleTiles();
    }

    RenderTilesEnum evaluateRenderTilesCameraXType() {
        auto xTileCamP = proportionXCamCordByCurrentExtent();
        if(xTileCamP > 1 - showTilesXCordProportion) {
            return RenderTilesEnum::SHOW_RIGHT;
        } else if(xTileCamP < showTilesXCordProportion) {
           return RenderTilesEnum::SHOW_LEFT;
        }
        return RenderTilesEnum::ONLY_MIDDLE;
    }

    std::shared_ptr<Symbols> getSymbols() {return symbols;}

    VisibleTileRenderMode visibleTileRenderMode = VisibleTileRenderMode::TILE_COORDINATES;

    void loadAssets(AAssetManager *assetManager);
    TileCords evaluateTileCords(int dX, int dY);
    void loadAndRenderCurrentVisibleTiles();
    void loadAndRender(TileCords tileCords);
private:
    short mapZTileCordMax = 19;
    float mapZTileScalingRoot = 12;
    float scaleFactorZoom = 0.0f; // из MapView устанавливается

    float _savedLastScaleStateMapZ;

    bool updateSavedLastDragXTileType() {
        RenderTilesEnum renderTilesEnumX = evaluateRenderTilesCameraXType();
        if(_savedDragRenderXTile != renderTilesEnumX) {
            _savedDragRenderXTile = renderTilesEnumX;
            return true;
        }
        return false;
    }

    void updateMapZoomScaleFactor(float scaleFactor) {
        scaleFactorZoom = scaleFactor;
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
    int* rootTileX, *rootTileY;

    Matrix4 projectionMatrix;

    const uint32_t extent = 4096;
    float dragFingerMapSpeed = 2.0f;

    RenderTilesEnum _savedDragRenderXTile = RenderTilesEnum::ONLY_MIDDLE;

    std::vector<TileCords> currentVisibleTiles = {};

    float showTilesXCordProportion;
    float fovy;
    float cameraRootZ;
    float camera[3] = {};
    int screenW, screenH;
    Cache* cache;
    TilesStorage tilesStorage = TilesStorage(cache);
    static const short rendererTilesSize = 8;
    TileForRenderer tilesForRenderer[rendererTilesSize] = {};
};


#endif //TUSA_RENDERER_H
