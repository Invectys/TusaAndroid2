if(TARGET mbgl-vendor-geometry)
    return()
endif()

add_library(
        mbgl-vendor-geometry INTERFACE
)

target_include_directories(
        mbgl-vendor-geometry SYSTEM
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}/geometry/include
)

set_target_properties(
        mbgl-vendor-geometry
        PROPERTIES
        INTERFACE_MAPBOX_NAME "geometry"
        INTERFACE_MAPBOX_URL "https://github.com/mapbox/geometry"
        INTERFACE_MAPBOX_AUTHOR "Mapbox"
        INTERFACE_MAPBOX_LICENSE ${CMAKE_CURRENT_LIST_DIR}/geometry/LICENSE.md
)