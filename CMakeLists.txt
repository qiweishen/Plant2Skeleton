cmake_minimum_required(VERSION 3.22)
project(Skeleton_Segmentation)

set(CMAKE_CXX_STANDARD 20) # Ranges library (C++20), std::ranges::reverse_view is used in Graph.cpp
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

if (APPLE)
    set(CMAKE_C_COMPILER /usr/bin/clang CACHE STRING "" FORCE)
    set(CMAKE_CXX_COMPILER /usr/bin/clang++ CACHE STRING "" FORCE)
endif ()


# TODO: Refine below OpenMP detection, it can be simplified
if (APPLE)
    if (CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/19.1.0/include")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/19.1.0/lib/libomp.dylib")
    endif ()
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/19.1.0/include")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/19.1.0/lib/libomp.dylib")
    endif ()
endif ()


find_package(OpenMP REQUIRED)
if (OpenMP_FOUND)
    message("OpenMP FOUND")
    message(STATUS "OpenMP found ${OpenMP_VERSION}")
endif ()

find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
    message("Eigen3 FOUND")
    message(STATUS "Find Eigen3 ${EIGEN3_VERSION}")
    include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

find_package(Boost REQUIRED)
if (Boost_FOUND)
    message("Boost FOUND")
    message(STATUS "Find Boost ${Boost_VERSION}")
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif ()

# TODO: Refine below Easy3D detection
set(easy3d_DIR "Easy3D/Release")
find_package(easy3d REQUIRED)
if (easy3d_FOUND)
    message("Easy3D FOUND")
    message(STATUS "Find Easy3D ${easy3d_VERSION}")
    include_directories(${easy3d_INCLUDE_DIRS})
    link_directories(${easy3d_LIBRARY_DIRS})
endif ()


add_subdirectory(SkelSeg/deps)


set(SOURCE_FILES
        SkelSeg/main.cpp
        SkelSeg/Skeleton.cpp
        SkelSeg/Graph.cpp
        SkelSeg/LOP.cpp
        SkelSeg/Tools.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE
        OpenMP::OpenMP_CXX
        easy3d::viewer
        geometry-central
        polyscope
        ${THIRD_PARTY_LIBS}
)

