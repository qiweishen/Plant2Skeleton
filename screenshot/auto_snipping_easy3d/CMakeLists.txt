cmake_minimum_required(VERSION 3.22...3.25)
project(AutoSnipping LANGUAGES CXX)

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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(Easy3D_BUILD_SHARED_LIBS        OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_TUTORIALS          OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_DOCUMENTATION      OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_TESTS              OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_PYTHON_BINDINGS    OFF CACHE BOOL "" FORCE)
set(Easy3D_ENABLE_CGAL              OFF CACHE BOOL "" FORCE)
set(Easy3D_ENABLE_QT                ON CACHE BOOL "" FORCE)
set(Easy3D_ENABLE_FFMPEG            OFF CACHE BOOL "" FORCE)
add_subdirectory(../../deps/Easy3D ${CMAKE_BINARY_DIR}/_deps/Easy3D)

add_executable(${PROJECT_NAME} main.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE
        RESOURCE_DIR="${CMAKE_SOURCE_DIR}/../../resources"
)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PRIVATE
        easy3d::viewer
        easy3d::core
        easy3d::fileio
        easy3d::util
        easy3d::renderer
)
