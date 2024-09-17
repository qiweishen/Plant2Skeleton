cmake_minimum_required(VERSION 3.22)
cmake_policy(SET CMP0072 NEW)
project(Skeleton_Segmentation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_C_COMPILER /usr/bin/clang CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER /usr/bin/clang++ CACHE STRING "" FORCE)

if (APPLE)
    if (CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/18.1.8/include")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/18.1.8/lib/libomp.dylib")
    endif ()
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/18.1.8/include")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/18.1.8/lib/libomp.dylib")
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


set(Easy3D_DIR "SkelSeg/3rd_party/Easy3D/Release")
find_package(Easy3D COMPONENTS viewer REQUIRED)
if (Easy3D_FOUND)
    message("Easy3D FOUND")
    message(STATUS "Found Easy3D ${Easy3D_VERSION}")
    include_directories(${Easy3D_INCLUDE_DIRS})
    link_directories(${Easy3D_LIBRARY_DIRS})
endif()


add_subdirectory(SkelSeg/deps/geometry-central)
add_subdirectory(SkelSeg/deps/polyscope)


add_executable(${PROJECT_NAME}
        SkelSeg/main.cpp
        SkelSeg/Skeleton.cpp
        SkelSeg/Graph.cpp
        SkelSeg/LOP.cpp
        SkelSeg/Tools.cpp
        SkelSeg/utility/Random.cpp

#        SkelSeg/main_dataset_1.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        OpenMP::OpenMP_CXX
        ${Boost_LIBRARIES}
        easy3d::viewer
        geometry-central
        polyscope
)
