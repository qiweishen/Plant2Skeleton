cmake_minimum_required(VERSION 3.22...3.25)
if(APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "13.3" CACHE STRING "Minimum macOS version" FORCE)
endif()
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
if(APPLE)
    set(LIBOMP_PREFIX "")

    # Homebrew installed prefix
    find_program(BREW_EXECUTABLE brew)
    if(BREW_EXECUTABLE)
        execute_process(
                COMMAND ${BREW_EXECUTABLE} --prefix libomp
                OUTPUT_VARIABLE LIBOMP_PREFIX
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
        )
    endif()

    # Some common install prefix
    if(NOT LIBOMP_PREFIX OR NOT EXISTS "${LIBOMP_PREFIX}")
        foreach(PATH
                "/opt/homebrew/opt/libomp"      # Apple Silicon Homebrew
                "/usr/local/opt/libomp"         # Intel Homebrew
                "/opt/local"                    # MacPorts
                "/usr/local")                   # Manually installed
            if(EXISTS "${PATH}/include/omp.h")
                set(LIBOMP_PREFIX "${PATH}")
                break()
            endif()
        endforeach()
    endif()

    if(NOT LIBOMP_PREFIX OR NOT EXISTS "${LIBOMP_PREFIX}/include/omp.h")
        message(FATAL_ERROR
                "libomp not found. Install with one of:\n"
                "  brew install libomp\n"
                "  port install libomp")
    endif()

    message(STATUS "Found libomp at: ${LIBOMP_PREFIX}")

    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I${LIBOMP_PREFIX}/include")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "${LIBOMP_PREFIX}/lib/libomp.dylib")
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I${LIBOMP_PREFIX}/include")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "${LIBOMP_PREFIX}/lib/libomp.dylib")
    endif()
endif()
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

target_compile_definitions(${PROJECT_NAME} PRIVATE
        ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

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
