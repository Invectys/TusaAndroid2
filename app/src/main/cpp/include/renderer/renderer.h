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

    bool RENDER_TILE_PALLET_TEST = false;

    void onSurfaceChanged(int w, int h);
    void onSurfaceCreated(AAssetManager *assetManager);
    void renderFrame();
    void updateVisibleTiles(bool render = true);
    void drag(float dx, float dy);
    void scale(float scaleFactor);
    void doubleTap();
    int normalizeCoordinate(int coordinate);

    float proportionXCamCordByCurrentExtent() {
        float x = evaluateXTileNMultiplyX();
        float proportion = CommonUtils::fract(x);
        return proportion;
    }

    float proportionYCamCordByCurrentExtent() {
        float y = evaluateYTileNMultiplyY();
        float proportion = CommonUtils::fract(y);
        return proportion;
    }

    float evaluateN() {
        return pow(2, currentMapZTile());
    }

    float evaluateXTileNMultiplyX() {
        int n = evaluateN();
        auto x = CommonUtils::longitudeRadToX(getCurrentLonRad());
        float xTile = x * n;
        return xTile;
    }

    float evaluateYTileNMultiplyY() {
        int n = evaluateN();
        auto rad = getCurrentLatRad();
        auto y = CommonUtils::latitudeRadToY(-rad);
        float yTile = y * n;
        return yTile;
    }

    float evaluateZeroZoomCameraDist() {
        return (-cameraRootZ * evaluateScaleFactorFormulaForZ(0));
    }

    TileCords getClearTileCord(int x, int y, int z, short rx, short ry) {
        return TileCords {
                x,
                y,
                x,
                y,
                z,
                false,
                false,
                rx,
                ry
        };
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

    // считает рут тайл за счет камеры, рут тайл используется для определения того что видит юзер
    int evaluateRootTileDiffX() {
        short z = currentMapZTile();
        if (z == 0 || z == 1) {
            return 0;
        }
        int xTile = evaluateXTileNMultiplyX();
        if(xTile == evaluateN())
            xTile--;
        LOGI("[ROOT] X root tile %d", xTile);
        return xTile;
    }

    int evaluateRootTileDiffY() {
        short z = currentMapZTile();
        if (z == 0 || z == 1) {
            return 0;
        }
        int yTile = evaluateYTileNMultiplyY();
        if(yTile == evaluateN())
            yTile--;
        LOGI("[ROOT] Y root tile %d", yTile);
        return yTile;
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

        LOGI("Current tile extent %f", evaluateCurrentExtent());
    }

    float evaluateBasicPlanetRadius() {
        return evaluateZeroZoomCameraDist() * 0.3f;
    }

    void updatePlanetGeometry() {
        sphere.generateSphereData3(250, 250, planetRadius,
                                   0, 0, M_PI);
    }

    Matrix4 calculateViewMatrix() {
        Matrix4 viewMatrix = Matrix4();
        viewMatrix.translate(0, 0, camera[2]);
        return viewMatrix;
    }

    RenderTilesEnum evaluateRenderTilesCameraXType() {
        auto xTileCamP = proportionXCamCordByCurrentExtent();

        if(xTileCamP > 1 - showTilesXCordProportion) {
            LOGI("[TILE_CAMP] xTileCamP %f SHOW_RIGHT", xTileCamP);
            return RenderTilesEnum::SHOW_RIGHT;
        } else {
            LOGI("[TILE_CAMP] xTileCamP %f SHOW_LEFT", xTileCamP);
            return RenderTilesEnum::SHOW_LEFT;
        }
    }

    RenderTilesEnum evaluateRenderTilesCameraYType() {
        auto yTileCamP = proportionYCamCordByCurrentExtent();
        if(yTileCamP > 1 - showTilesXCordProportion) {
            LOGI("[TILE_CAMP] yTileCamP %f SHOW_BOTTOM", yTileCamP);
            return RenderTilesEnum::SHOW_BOTTOM;
        } else {
            LOGI("[TILE_CAMP] yTileCamP %f SHOW_TOP", yTileCamP);
            return RenderTilesEnum::SHOW_TOP;
        }
    }

    std::shared_ptr<Symbols> getSymbols() {return symbols;}

    VisibleTileRenderMode visibleTileRenderMode = VisibleTileRenderMode::TILE;

    void drawPlanet();
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

    TileCords topLeftVisibleCord;
    TileCords bottomRightVisibleCord;

    unsigned int testTextureId;

    void updateRenderTileProjection(short amountX, short amountY);

    bool updateSavedLastDragXTileType() {
        RenderTilesEnum renderTilesEnumX = evaluateRenderTilesCameraXType();
        if(_savedDragRenderXTile != renderTilesEnumX) {
            _savedDragRenderXTile = renderTilesEnumX;
            return true;
        }
        return false;
    }

    bool updateSavedLastDragYTileType() {
        RenderTilesEnum renderTilesEnumY = evaluateRenderTilesCameraYType();
        if(_savedDragRenderYTile != renderTilesEnumY) {
            _savedDragRenderYTile = renderTilesEnumY;
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

    float getCurrentLonRad() {
        return DEG2RAD(getPlanetCurrentLongitude());
    }

    float getCurrentLatRad() {
        return DEG2RAD(getPlanetCurrentLatitude());
    }

    float getPlanetCurrentLongitude(float xDeg) {
        float result = fmod(xDeg + 180.0, 360.0);
        if (result < 0) {
            result += 360.0;
        }
        return result - 180.0;
    }

    float getPlanetCurrentLongitude() {
        return getPlanetCurrentLongitude(cameraXAngleDeg);
    }

    float getPlanetCurrentLatitude() {
        return cameraYAngleDeg;
    }

    void drawPoint(Matrix4 matrix, float x, float y, float z) {
        auto errorstr = CommonUtils::getGLErrorString();
        auto plainShader = shadersBucket.get()->plainShader;
        float points[] = {x, y, z};
        const GLfloat color[] = { 1, 0, 0, 1};
        glUseProgram(plainShader->program);
        glUniform4fv(plainShader->getColorLocation(), 1, color);
        glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, matrix.get());
        glVertexAttribPointer(plainShader->getPosLocation(), 3, GL_FLOAT,
                              GL_FALSE, 0, points
        );
        glEnableVertexAttribArray(plainShader->getPosLocation());
        glDrawArrays(GL_POINTS, 0, 1);
        errorstr = CommonUtils::getGLErrorString();
    }


    std::shared_ptr<ShadersBucket> shadersBucket;
    RenderTileGeometry renderTileGeometry;
    std::shared_ptr<RenderTileCoordinates> renderTileCoordinates;
    std::shared_ptr<Symbols> symbols;
    int* rootTileX, *rootTileY;

    Matrix4 projectionMatrix;
    Matrix4 rendererTileProjectionMatrix;
    Matrix4 rendererTileProjectionMatrixUI_TEST;

    const uint32_t extent = 4096;
    float dragFingerMapSpeed = 0.00002f;

    RenderTilesEnum _savedDragRenderXTile = RenderTilesEnum::ONLY_MIDDLE;
    RenderTilesEnum _savedDragRenderYTile = RenderTilesEnum::ONLY_MIDDLE;

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

    float renderXDiffSize = 0;
    float renderYDiffSize = 0;

    Cache* cache;
    TilesStorage tilesStorage = TilesStorage(cache);
    static const short rendererTilesSize = 30;
    TileForRenderer tilesForRenderer[rendererTilesSize] = {};
    std::vector<float> cord_tiles[rendererTilesSize] = {};

    GLuint renderTexture[rendererTilesSize] = {};
    GLuint frameBuffer;
    GLuint depthBuffer;
};


#endif //TUSA_RENDERER_H
