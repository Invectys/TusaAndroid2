if(TARGET mbgl-vendor-variant)
    return()
endif()

add_library(
        mbgl-vendor-variant INTERFACE
)

target_include_directories(
        mbgl-vendor-variant SYSTEM
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}/variant/include
)

set_target_properties(
        mbgl-vendor-variant
        PROPERTIES
        INTERFACE_MAPBOX_NAME "variant"
        INTERFACE_MAPBOX_URL "https://github.com/mapbox/variant"
        INTERFACE_MAPBOX_AUTHOR "Mapbox"
        INTERFACE_MAPBOX_LICENSE ${CMAKE_CURRENT_LIST_DIR}/variant/LICENSE.md
)