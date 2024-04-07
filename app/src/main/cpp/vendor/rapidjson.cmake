if(TARGET rapidjson)
    return()
endif()

add_library(
        rapidjson INTERFACE
)

target_include_directories(
        rapidjson SYSTEM
        INTERFACE ${CMAKE_CURRENT_LIST_DIR}/rapidjson/include
)

set_target_properties(
        rapidjson
        PROPERTIES
        INTERFACE_MAPBOX_NAME "rapidjson"
        INTERFACE_MAPBOX_URL "https://github.com/mapbox/rapidjson"
        INTERFACE_MAPBOX_AUTHOR "Mapbox"
        INTERFACE_MAPBOX_LICENSE ${CMAKE_CURRENT_LIST_DIR}/rapidjson/lincense.txt
)