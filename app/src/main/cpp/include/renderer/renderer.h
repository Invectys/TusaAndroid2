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
#include "center_borders_cords.h"
#include "corners_cords.h"
#include "util/eigen_gl.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <queue>
#include <stack>


class Renderer {
public:
    Renderer(std::shared_ptr<ShadersBucket> shadersBucket, Cache* cache);
    ~Renderer() {}


    bool RENDER_TILE_PALLET_TEST = false;

    void onSurfaceChanged(int w, int h);
    void onSurfaceCreated(AAssetManager *assetManager);
    void renderFrame();
    void updateVisibleTiles();
    void drag(float dx, float dy);
    void scale(float scaleFactor);
    void doubleTap();
    void networkTilesFunction(JavaVM* gJvm, GetTileRequest* getTileRequest);

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
        return (cameraRootDistance * evaluateScaleFactorFormulaForZ(0));
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

    void setupNoOpenGLMapState(float scaleFactor, AAssetManager *assetManager, JNIEnv *env);

    float evaluateBasicPlanetRadius() {
        return evaluateZeroZoomCameraDist() * 0.3f;
    }

    void updatePlanetGeometry();

    std::shared_ptr<Symbols> getSymbols() {return symbols;}

    VisibleTileRenderMode visibleTileRenderMode = VisibleTileRenderMode::TILE;

    void drawPlanet();
    void onStop();
    void loadAssets(AAssetManager *assetManager);
    void loadAndRenderCurrentVisibleTiles();
    void loadAndRender(TileCords needToInsertCord, GetTileRequest* getTileRequest);

    void updateMapZoomScaleFactor(float scaleFactor) {
        scaleFactorZoom = scaleFactor;
    }

    void updateCameraPosition() {
        cameraCurrentDistance = cameraRootDistance * evaluateFloatScaleFactorFormula();
    }

    float evaluateCameraZByZoomingBorder(short zoomDiff = 0) {
        return cameraRootDistance * evaluateScaleFactorFormula(zoomDiff);
    }

    void loadTextures(AAssetManager *assetManager);

private:

    short networkTilesThreads = 2;
    short mapZTileCordMax = 19;
    float mapZTileScalingRoot = 12;
    float scaleFactorZoom = 0.0f; // из MapView устанавливается
    float cameraCurrentDistance = 0;

    std::stack<TileCords> networkTilesStack;
    std::vector<std::thread*> networkTileThreads;
    float _lastEvaluatedScaleFloatScaleFactor;
    float _savedLastScaleStateMapZ;

    TileCords topLeftVisibleCord;
    TileCords bottomRightVisibleCord;

    unsigned int testTextureId;

    void updateRenderTileProjection(short amountX, short amountY);

    void updateFrustum();

    Eigen::Matrix4f evaluatePVM() {
        float camX = -1 * (cameraCurrentDistance + planetRadius);
        Eigen::Vector3f cameraPosition = Eigen::Vector3f(camX, 0, 0);
        Eigen::AngleAxis<float> rotateMatrixUnitY =
                Eigen::AngleAxis<float>(cameraLongitudeRad, Eigen::Vector3f(0, 1, 0));

        Eigen::AngleAxis<float> rotateMatrixUnitZ =
                Eigen::AngleAxis<float>(cameraLatitudeRad, Eigen::Vector3f(0, 0, 1));

        cameraPosition = rotateMatrixUnitY * rotateMatrixUnitZ * cameraPosition;
        Eigen::Vector3f target(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
        Eigen::Matrix4f viewMatrix = EigenGL::createViewMatrix(cameraPosition, target, up);
        Eigen::Matrix4f pvm = projectionMatrix * viewMatrix;
        return pvm;
    }

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

    float getPlanetCurrentLongitude() {
        return RAD2DEG(CommonUtils::normalizeLongitudeRad(cameraLongitudeRad));
    }

    float getPlanetCurrentLatitude() {
        return RAD2DEG(cameraLatitudeRad);
    }

    CornersCords evaluateCorners(Eigen::Matrix4f pvm);
    void evaluateLatLonByIntersectionPlanes(Eigen::Vector4f firstPlane, Eigen::Vector4f secondPlane, Eigen::Matrix4f pvm, float& longitudeRad, float& latitudeRad, bool& has);

    void drawPoints(Eigen::Matrix4f matrix, std::vector<float> points, float pointSize) {
        glDisable(GL_STENCIL_TEST);
        auto plainShader = shadersBucket.get()->plainShader;
        const GLfloat color[] = { 1, 0, 0, 1};
        glUseProgram(plainShader->program);
        glUniform1f(plainShader->getPointSizeLocation(), pointSize);
        glUniform4fv(plainShader->getColorLocation(), 1, color);
        glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, matrix.data());
        glVertexAttribPointer(plainShader->getPosLocation(), 3, GL_FLOAT, GL_FALSE, 0, points.data());
        glEnableVertexAttribArray(plainShader->getPosLocation());
        glDrawArrays(GL_POINTS, 0, points.size() / 3);
        glEnable(GL_STENCIL_TEST);
    }

    void drawPoint(Eigen::Matrix4f matrix, float x, float y, float z, float pointSize) {
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_STENCIL_TEST);
        auto plainShader = shadersBucket.get()->plainShader;
        glUniform1f(plainShader->getPointSizeLocation(), pointSize);
        auto errorstr = CommonUtils::getGLErrorString();
        float points[] = {x, y, z};
        const GLfloat color[] = { 1, 0, 0, 1};
        glUseProgram(plainShader->program);
        glUniform4fv(plainShader->getColorLocation(), 1, color);
        glUniformMatrix4fv(plainShader->getMatrixLocation(), 1, GL_FALSE, matrix.data());
        glVertexAttribPointer(plainShader->getPosLocation(), 3, GL_FLOAT,
                              GL_FALSE, 0, points
        );
        glEnableVertexAttribArray(plainShader->getPosLocation());
        glDrawArrays(GL_POINTS, 0, 1);
        errorstr = CommonUtils::getGLErrorString();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_STENCIL_TEST);
    }

    void drawVerticesPlanetCoordinates(Eigen::Matrix4f pvm) {
        //glDisable(GL_DEPTH_TEST);
        glDisable(GL_STENCIL_TEST);
        for(int i = 0; i < sphere.sphere_vertices.size(); i += 3) {
            auto t = sphere.unitSquareCoordinates[i / 3 * 2];
            auto s = sphere.unitSquareCoordinates[i / 3 * 2 + 1];
            auto x = sphere.sphere_vertices[i];
            auto y = sphere.sphere_vertices[i + 1];
            auto z = sphere.sphere_vertices[i + 2];
            Matrix4 textModelMatrix = Matrix4();
            textModelMatrix.rotate(cameraLongitudeRad - 90, 0, -1, 0);
            textModelMatrix.rotate(cameraLatitudeRad, 1, 0, 0);
            textModelMatrix.translate(0, 0, -planetRadius);

//            Matrix4 textMatrix = pvm * textModelMatrix;
//            std::string text = CommonUtils::formatFloat(t) + " " + CommonUtils::formatFloat(s);
//            symbols.get()->renderText2(text, textMatrix, x, y, z, 500);
        }
        //glEnable(GL_DEPTH_TEST);
        glEnable(GL_STENCIL_TEST);
    }

    bool isCurrentVisible(TileCords& tileCords) {
        for (auto cv : currentVisibleTiles) {
            if (tileCords.tileX == cv.tileX && tileCords.tileY == cv.tileY && tileCords.tileZ == cv.tileZ) {
                return true;
            }
        }
        return false;
    }

    void glViewPortDefaultSize() {
        glViewport(0, 0, screenW, screenH);
    }

    std::shared_ptr<ShadersBucket> shadersBucket;
    RenderTileGeometry renderTileGeometry;
    std::shared_ptr<RenderTileCoordinates> renderTileCoordinates;
    std::shared_ptr<Symbols> symbols;

    Eigen::Matrix4f projectionMatrix;
    Eigen::Matrix4f rendererTileProjectionMatrix;

    const uint32_t extent = 4096;
    double dragFingerMapSpeed = 0.0000002;

    std::vector<TileCords> currentVisibleTiles = {};


    Sphere sphere = Sphere();
    float planetRadius;
    double latitudeCameraAngleRadConstraint = M_PI / 2 - M_PI / 100;
    float fovy;
    float cameraRootDistance;
    double cameraLatitudeRad = 0, cameraLongitudeRad = 0;
    int screenW, screenH;

    int renderMapTextureWidth = extent, renderMapTextureHeight = extent;

    Cache* cache;
    TilesStorage tilesStorage = TilesStorage(cache, nullptr);
    static const short tilesForRenderMaxSize = 20;
    TileForRenderer tilesForRenderer[tilesForRenderMaxSize] = {};

    short renderMapXTilesCount, renderMapYTilesCount;
    GLuint renderMapTexture;
    GLuint renderMapFrameBuffer;
    GLuint renderMapDepthBuffer;
};


#endif //TUSA_RENDERER_H
