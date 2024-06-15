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
#include "util/android_log.h"
#include "geometry/sphere.h"
#include "gl/triangle_example.h"
#include <vector>


class Renderer {
public:
    Renderer(std::shared_ptr<ShadersBucket> shadersBucket, Cache* cache);
    ~Renderer() {
        delete rootTileX;
        delete rootTileY;
    }

    void onSurfaceChanged(int w, int h);
    void onSurfaceCreated(AAssetManager *assetManager);
    void renderFrame();
    void updateVisibleTiles(bool render = true);
    void drag(float dx, float dy);
    void scale(float scaleFactor);
    void doubleTap();
    int normalizeCoordinate(int coordinate);

    float proportionXCamCordByCurrentExtent() {
        auto tileRemain = fmod(-camera[0], evaluateCurrentExtent());
        return tileRemain / evaluateCurrentExtent();
    }

    float evaluateZeroZoomCameraDist() {
        return (-cameraRootZ * evaluateScaleFactorFormulaForZ(0));
    }

    // Устанавливает камеру в центр тайла
    void updateCordsByTiles(int xTile, int yTile) {
        *rootTileX = xTile;
        *rootTileY = yTile;
        auto _currentExtent = evaluateCurrentExtent();
        auto cameraXStart = - *rootTileX * _currentExtent - _currentExtent / 2;
        auto cameraYStart = *rootTileY * _currentExtent + _currentExtent;
        camera[0] = cameraXStart;
        camera[1] = cameraYStart;
    }

    int evaluateRootTileDiffX() {
        return -1 * camera[0] / evaluateCurrentExtent();
    }

    int evaluateRootTileDiffY() {
        auto _currentExtent = evaluateCurrentExtent();
        return (camera[1] - _currentExtent / 2) / _currentExtent;
    }

    void updateRootTile() {
        *rootTileX = evaluateRootTileDiffX();
        *rootTileY = evaluateRootTileDiffY();
    }

    void setupNoOpenGLMapState(float scaleFactor, AAssetManager *assetManager) {
        showTilesXCordProportion = 0.3;
        rootTileX = new int(0);
        rootTileY = new int(0);
        loadAssets(assetManager);
        cameraRootZ = -3500;
        fovy = 65;
        updateMapZoomScaleFactor(scaleFactor);
        //updateCordsByTiles(0, 0);
        camera[0] = 0;
        camera[1] = 0;
        updateMapCameraZPosition();
        _savedLastScaleStateMapZ = currentMapZTile();
        renderTileGeometry.scaleZCordDrawHeapsDiff(evaluateScaleFactorFormula());
        planetRadius = evaluateBasicPlanetRadius();
        updatePlanetGeometry();
        updateVisibleTiles();




        LOGI("Current tile extent %f", evaluateCurrentExtent());
    }

    float evaluateBasicPlanetRadius() {
        return evaluateZeroZoomCameraDist() * 0.3f;
    }

    void updatePlanetGeometry() {
        sphere.generateSphereData3(25, 25, planetRadius, 0, 0, M_PI);
    }

    RenderTilesEnum evaluateRenderTilesCameraXType() {
        auto xTileCamP = proportionXCamCordByCurrentExtent();
        LOGI("xTileCamP %f", xTileCamP);
        if(xTileCamP > 1 - showTilesXCordProportion) {
            return RenderTilesEnum::SHOW_RIGHT;
        } else if(xTileCamP < showTilesXCordProportion) {
           return RenderTilesEnum::SHOW_LEFT;
        }
        return RenderTilesEnum::ONLY_MIDDLE;
    }

    std::shared_ptr<Symbols> getSymbols() {return symbols;}

    VisibleTileRenderMode visibleTileRenderMode = VisibleTileRenderMode::TILE;

    void drawPlanet();
    void drawTile(TileForRenderer visibleTile, Matrix4 pvmTexture, std::vector<short> possibleDeltasVec);
    void loadAssets(AAssetManager *assetManager);
    TileCords evaluateTileCords(int dX, int dY);
    void loadAndRenderCurrentVisibleTiles();
    void loadAndRender(TileCords tileCords);

    void updateMapZoomScaleFactor(float scaleFactor) {
        scaleFactorZoom = scaleFactor;
    }

    void updateMapCameraZPosition() {
        camera[2] = cameraRootZ * evaluateFloatScaleFactorFormula();
    }

    float evaluateCameraZByZoomingBorder(short zoomDiff = 0) {
        return cameraRootZ * evaluateScaleFactorFormula(zoomDiff);
    }

    void loadTextures(AAssetManager *assetManager);

private:
    GLuint sphereVBO, sphereIBO;

private:
    short mapZTileCordMax = 19;
    float mapZTileScalingRoot = 12;
    float scaleFactorZoom = 0.0f; // из MapView устанавливается
    short maxLoadTileThreads = 4;
    short loadTilesThreadsAmount = 0;

    float _lastEvaluatedScaleFloatScaleFactor;
    float _savedLastScaleStateMapZ;

    unsigned int testTextureId;

    void updateRenderTileProjection(int amountX, int amountY);

    bool updateSavedLastDragXTileType() {
        RenderTilesEnum renderTilesEnumX = evaluateRenderTilesCameraXType();
        if(_savedDragRenderXTile != renderTilesEnumX) {
            _savedDragRenderXTile = renderTilesEnumX;
            return true;
        }
        return false;
    }

    void updateFrustum();

    float evaluateFloatScaleFactorFormula() {
        _lastEvaluatedScaleFloatScaleFactor = pow(2, mapZTileScalingRoot - scaleFactorZoom);
        return _lastEvaluatedScaleFloatScaleFactor;
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

    float evaluateCurrentExtent() {
        return evaluateExtentForScaleFactor(evaluateScaleFactorFormula());
    }

    float getPlanetCurrentLongitude() {
        return cameraXAngleDeg;
    }

    float getPlanetCurrentLatitude() {
        return cameraYAngleDeg;
    }


    std::shared_ptr<ShadersBucket> shadersBucket;
    RenderTileGeometry renderTileGeometry;
    std::shared_ptr<RenderTileCoordinates> renderTileCoordinates;
    std::shared_ptr<Symbols> symbols;
    int* rootTileX, *rootTileY;

    Matrix4 projectionMatrix;
    Matrix4 rendererTileProjectionMatrix;

    const uint32_t extent = 4096;
    float dragFingerMapSpeed = 0.00002f;

    RenderTilesEnum _savedDragRenderXTile = RenderTilesEnum::ONLY_MIDDLE;

    std::vector<TileCords> currentVisibleTiles = {};


    Sphere sphere = Sphere();
    float planetRadius;
    float degLatitudeCameraConstraint = 70;
    std::vector<TileCords> toShowTilesQueue = {};
    float showTilesXCordProportion;
    float fovy;
    float cameraRootZ;
    float camera[3] = {};
    float cameraYAngleDeg = 0;
    float cameraXAngleDeg = 0;
    int screenW, screenH;
    Cache* cache;
    TilesStorage tilesStorage = TilesStorage(cache);
    static const short rendererTilesSize = 4;
    TileForRenderer tilesForRenderer[rendererTilesSize] = {};
    std::vector<float> cord_tiles[rendererTilesSize] = {};

    GLuint renderTexture[rendererTilesSize] = {};
    GLuint frameBuffer;
    GLuint depthBuffer;
};


#endif //TUSA_RENDERER_H
