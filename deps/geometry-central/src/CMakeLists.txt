file(GLOB_RECURSE GC_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

add_library(geometry-central ${GC_SOURCES})

target_include_directories(geometry-central PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# OpenMP is a compiler feature; use find_package
if(APPLE)
    execute_process(
            COMMAND brew --prefix
            OUTPUT_VARIABLE BREW_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(OpenMP_ROOT "${BREW_PREFIX}/opt/libomp")
endif()
find_package(OpenMP COMPONENTS CXX REQUIRED)

target_link_libraries(geometry-central PRIVATE
        happly
        nanoflann
        nanort
        OpenMP::OpenMP_CXX
        Eigen
)
