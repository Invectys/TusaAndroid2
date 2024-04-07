if(TARGET vtzero-vendor)
    return()
endif()

add_library(
        vtzero-vendor INTERFACE
)

target_include_directories(
        vtzero-vendor SYSTEM
        INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/vtzero/include/vtzero
        ${CMAKE_CURRENT_LIST_DIR}/vtzero/include-external
)

set_target_properties(
        vtzero-vendor
        PROPERTIES
        INTERFACE_MAPBOX_NAME "vtzero"
        INTERFACE_MAPBOX_URL "https://github.com/mapbox/vtzero"
        INTERFACE_MAPBOX_AUTHOR "Mapbox"
        INTERFACE_MAPBOX_LICENSE ${CMAKE_CURRENT_LIST_DIR}/vtzero/LICENSE.md
)