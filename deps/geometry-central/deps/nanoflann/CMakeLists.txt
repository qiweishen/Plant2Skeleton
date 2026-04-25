add_library(nanoflann INTERFACE)

target_include_directories(nanoflann SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
