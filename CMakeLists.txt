cmake_minimum_required(VERSION 3.22...3.25)
project(SkelSeg LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20) # Ranges library (C++20), std::ranges is used in this project
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

####################################################################################################
####################################################################################################
# OpenMP is a compiler feature; use find_package
find_package(OpenMP COMPONENTS CXX REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found: ${OpenMP_VERSION}")
endif()

# Find Easy3D; consider isolating Easy3D in a separate CMakeLists.txt
set(Easy3D_DIR "/usr/local/easy3d-2.6.1/lib/CMake")
find_package(Easy3D REQUIRED)
if(Easy3D_FOUND)
    message(STATUS "Easy3D found: ${Easy3D_VERSION}")
endif()

# Find Boost; consider isolating Boost in a separate CMakeLists.txt
find_package(Boost COMPONENTS graph REQUIRED)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_VERSION}")
endif()
####################################################################################################
####################################################################################################

add_subdirectory(deps)

include_directories(
        ${PROJECT_SOURCE_DIR}/SkelSeg/include
)

set(SOURCES
        SkelSeg/src/main.cpp
        SkelSeg/src/skeleton.cpp
        SkelSeg/src/lop.cpp
        SkelSeg/src/graph.cpp
        SkelSeg/src/tools.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PUBLIC
        OpenMP::OpenMP_CXX
        Boost::graph
        easy3d::viewer
        Eigen
        geometry-central
        KDTree
        nlohmann
        plywoot
)
