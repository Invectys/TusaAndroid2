# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html.
# For more examples on how to use CMake, see https://github.com/android/ndk-samples.

# Sets the minimum CMake version required for this project.
cmake_minimum_required(VERSION 3.22.1)

# Declares the project name. The project name can be accessed via ${ PROJECT_NAME},
# Since this is the top level freetype.cmake, the project name is also accessible
# with ${CMAKE_PROJECT_NAME} (both CMake variables are in-sync within the top level
# build script scope).
project("tusaandroid")

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
#
# In this top level freetype.cmake, ${CMAKE_PROJECT_NAME} is used to define
# the target library name; in the sub-module's freetype.cmake, ${PROJECT_NAME}
# is preferred for the same purpose.
#
# In order to load a library into your app from Java/Kotlin, you must call
# System.loadLibrary() and pass the name of the library defined here;
# for GameActivity/NativeActivity derived applications, the same library name must be
# used in the AndroidManifest.xml file.
add_library(${CMAKE_PROJECT_NAME} SHARED
        # List C/C++ source files with relative paths to this freetype.cmake.
        native-lib.cpp

        shader/plain_shader.cpp
        shader/symbol_shader.cpp
        shader/shaders_bucket.cpp
        shader/shader.cpp
        shader/planet_shader.cpp

        renderer/renderer.cpp
        renderer/map_draw_call.cpp
        renderer/render_tile_geometry.cpp
        renderer/render_tile_coordinates.cpp
        renderer/render_tiles_enum.cpp
        renderer/tile_cords.cpp
        renderer/center_borders_cords.cpp
        renderer/corners_cords.cpp

        util/frustrums.cpp
        util/matrices.cpp
        util/timer.cpp
        util/eigen_gl.cpp

        network/get_tile_request.cpp

        gl/open_gl_interface.cpp

        map/map.cpp
        map/tile.cpp
        map/tiles_storage.cpp
        map/tile_for_renderer.cpp
        map/linestring_handler.cpp
        map/polygon_handler.cpp
        map/visible_tile_render_mode.cpp
        map/mercator.cpp

        geometry/geometry.cpp
        geometry/sphere.cpp

        cache/cache.cpp

        style/style.cpp
        style/style_expecter.cpp

        symbols/symbols.cpp
        symbols/symbol.cpp

        csscolorparser/csscolorparser.cpp
)

add_subdirectory(${PROJECT_SOURCE_DIR}/eigen)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE include)

include(${PROJECT_SOURCE_DIR}/vendor/protozero.cmake)
#include(${PROJECT_SOURCE_DIR}/vendor/vector-tile.cmake)
include(${PROJECT_SOURCE_DIR}/vendor/earcut.hpp.cmake)
include(${PROJECT_SOURCE_DIR}/vendor/boost.cmake)
#include(${PROJECT_SOURCE_DIR}/vendor/geometry.cmake)
include(${PROJECT_SOURCE_DIR}/vendor/variant.cmake)
include(${PROJECT_SOURCE_DIR}/vendor/curl.cmake)
include(${PROJECT_SOURCE_DIR}/vendor/rapidjson.cmake)
include(${PROJECT_SOURCE_DIR}/vendor/vtzero.cmake)
include(${PROJECT_SOURCE_DIR}/vendor/freetype.cmake)

# Specifies libraries CMake should link to your target library. You
# can link libraries from various origins, such as libraries defined in this
# build script, prebuilt third-party libraries, or Android system libraries.
target_link_libraries(${CMAKE_PROJECT_NAME}
        mbgl-vendor-protozero
        #        mbgl-vendor-vector-tile
        mbgl-vendor-earcut.hpp
        mbgl-vendor-boost
        #        mbgl-vendor-geometry
        mbgl-vendor-variant
        vtzero-vendor
        rapidjson
        curl
        freetype
        Eigen3::Eigen
        )

target_link_libraries(${CMAKE_PROJECT_NAME} log)
target_link_libraries(${CMAKE_PROJECT_NAME} GLESv2)
target_link_libraries(${CMAKE_PROJECT_NAME} android)
target_link_libraries(${CMAKE_PROJECT_NAME} z)
target_link_libraries(${CMAKE_PROJECT_NAME} nghttp2)
target_link_libraries(${CMAKE_PROJECT_NAME} ssl crypto)