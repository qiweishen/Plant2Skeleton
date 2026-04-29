cmake_minimum_required(VERSION 3.22...3.25)
project(SkelSeg LANGUAGES CXX)

# Ranges library (C++20), std::ranges is used in this project
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CMake policies
if (POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
    set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)
endif ()

####################################################################################################
####################################################################################################
# OpenMP is a compiler feature; use find_package
find_package(OpenMP COMPONENTS CXX REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found: ${OpenMP_VERSION}")
endif()

# Find Boost; consider isolating Boost in a separate CMakeLists.txt
find_package(Boost COMPONENTS graph REQUIRED)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_VERSION}")
endif()
####################################################################################################
####################################################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(deps)

include_directories(
        ${PROJECT_SOURCE_DIR}/SkelSeg/include
)

####################################################################################################
# SkelSeg static library (all sources except main.cpp)
####################################################################################################
set(LIB_SOURCES
        SkelSeg/src/skeleton.cpp
        SkelSeg/src/lop.cpp
        SkelSeg/src/graph.cpp
        SkelSeg/src/tools.cpp
)

add_library(SkelSeg_lib STATIC ${LIB_SOURCES})

target_include_directories(SkelSeg_lib PUBLIC
        ${PROJECT_SOURCE_DIR}/SkelSeg/include
)

target_link_libraries(SkelSeg_lib PUBLIC
        OpenMP::OpenMP_CXX
        Boost::graph
        easy3d::viewer
        easy3d::core
        easy3d::fileio
        easy3d::util
        easy3d::renderer
        Eigen
        fmt::fmt
        geometry-central
        KDTree
        nlohmann
        plywoot
)

####################################################################################################
# SkelSeg command-line executable
####################################################################################################
add_executable(${PROJECT_NAME} SkelSeg/src/main.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
        SkelSeg_lib
)

####################################################################################################
# GUI Viewer
####################################################################################################
add_subdirectory(app/viewer)

####################################################################################################
# Skeleton Evaluator
####################################################################################################
add_subdirectory(eval/skeleton)
