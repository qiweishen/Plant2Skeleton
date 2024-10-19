cmake_minimum_required(VERSION 3.22...3.25)
project(Skeleton_Segmentation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20) # Ranges library (C++20), std::ranges is used in this project
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

####################################################################################################
####################################################################################################
# OpenMP is a compiler feature; use find_package
if(APPLE)
    execute_process(
            COMMAND brew --prefix
            OUTPUT_VARIABLE BREW_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(OpenMP_ROOT "${BREW_PREFIX}/opt/libomp")
endif()
find_package(OpenMP COMPONENTS CXX)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found: ${OpenMP_VERSION}")
else()
    set(OpenMP_FLAGS )
    message(WARNING "OpenMP not found, proceeding without OpenMP support")
endif()

# Find Easy3D; consider isolating Easy3D in a separate CMakeLists.txt
set(easy3d_DIR "SkelSeg/3rd_party/easy3d/Release")
find_package(easy3d REQUIRED)
if(easy3d_FOUND)
    message(STATUS "Easy3D found: ${easy3d_VERSION}")
endif()

# Find Boost; consider isolating Boost in a separate CMakeLists.txt
find_package(Boost COMPONENTS graph REQUIRED)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_VERSION}")
endif()
####################################################################################################
####################################################################################################

add_subdirectory(SkelSeg/deps)

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
