if (TARGET curl)
    return()
endif ()

add_library(curl STATIC IMPORTED)

target_include_directories(
        curl
        INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/curl/include
)

set_target_properties(
        curl
        PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/curl/${ANDROID_ABI}/libcurl.a
)
