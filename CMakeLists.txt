cmake_minimum_required(VERSION 3.22...3.25)
project(Skeleton_Segmentation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20) # Ranges library (C++20), std::ranges::reverse_view is used in Graph.cpp
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#set(CMAKE_C_COMPILER /usr/bin/clang CACHE STRING "" FORCE)
#set(CMAKE_CXX_COMPILER /usr/bin/clang++ CACHE STRING "" FORCE)
#
### TODO: Refine below OpenMP detection, it can be simplified
#if (APPLE)
#    if (CMAKE_C_COMPILER_ID MATCHES "Clang\$")
#        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/19.1.1/include")
#        set(OpenMP_C_LIB_NAMES "omp")
#        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/19.1.1/lib/libomp.dylib")
#    endif ()
#    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
#        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/19.1.1/include")
#        set(OpenMP_CXX_LIB_NAMES "omp")
#        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/19.1.1/lib/libomp.dylib")
#    endif ()
#endif ()
#find_package(OpenMP REQUIRED)
#if (OpenMP_FOUND)
#    message("OpenMP FOUND")
#    message(STATUS "OpenMP found ${OpenMP_VERSION}")
#endif ()

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
if(OpenMP_FOUND)
    message(STATUS "OpenMP found: ${OpenMP_VERSION}")
else()
    message(WARNING "OpenMP not found, proceeding without OpenMP support")
endif()

# Find Easy3D; consider isolating Easy3D in a separate CMakeLists.txt
set(easy3d_DIR "Easy3D/Release")
find_package(easy3d REQUIRED)
if(easy3d_FOUND)
    message(STATUS "Easy3D found: ${easy3d_VERSION}")
endif()

# Find Boost; consider isolating Boost in a separate CMakeLists.txt
find_package(Boost COMPONENTS graph REQUIRED)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_VERSION}")
endif()

add_subdirectory(SkelSeg/deps)

# Include directories
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
        Eigen
        Boost::graph
        easy3d::viewer
        fast_float
        geometry-central
        nlohmann
        plywoot
)
