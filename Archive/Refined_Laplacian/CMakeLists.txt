cmake_minimum_required(VERSION 3.22)
project(CppTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Debug")

if(APPLE)
    set(CMAKE_C_COMPILER clang)
    set(CMAKE_CXX_COMPILER clang++)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/18.1.6/include")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/18.1.6/lib/libomp.dylib")
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/18.1.6/include")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/18.1.6/lib/libomp.dylib")
    endif()
    set(Open3D_DIR "/Users/shenqiwei/open3d/lib/cmake/Open3D")
    set(Easy3D_DIR "/Users/shenqiwei/Easy3D/Release")
else ()
    set(Open3D_DIR "/Users/shenqiwei/open3d/lib/cmake/Open3D")
    set(Easy3D_DIR "/Users/shenqiwei/Easy3D/Release")
endif()


find_package(OpenMP REQUIRED)
if (OpenMP_FOUND)
    message("OpenMP FOUND")
    message(STATUS "OpenMP found ${OpenMP_VERSION}")
endif()


find_package(Open3D REQUIRED)
if (Open3D_FOUND)
    message("Open3D FOUND")
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    include_directories(${Open3D_INCLUDE_DIRS})
    link_directories(${Open3D_LIBRARY_DIRS})
    include_directories(/Users/shenqiwei/open3d/include/open3d/3rdparty)
endif()


find_package(Easy3D REQUIRED)
if (Easy3D_FOUND)
    message("Easy3D FOUND")
    message(STATUS "Found Easy3D ${Easy3D_VERSION}")
    include_directories(${Easy3D_INCLUDE_DIRS})
    link_directories(${Easy3D_LIBRARY_DIRS})
endif()


find_package(CGAL REQUIRED)
if (CGAL_FOUND)
    message("CGAL FOUND")
    message(STATUS "Find CGAL ${CGAL_VERSION}")
    include_directories(${CGAL_INCLUDE_DIRS})
    link_directories(${CGAL_LIBRARY_DIRS})
    add_definitions(${CGAL_DEFINITIONS})
endif ()
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)


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


add_executable(${PROJECT_NAME}
        Skeleton_refined.cpp
        Tools_refined.cpp
        main_refined.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        OpenMP::OpenMP_CXX
        Open3D::Open3D
        ${Open3D_LIBRARY}
        ${Easy3D_LIBRARY}
        ${CGAL_LIBRARIES}
        ${CGAL_3RD_PARTY_LIBRARIES}
        ${TBB_LIBRARIES}
        ${Boost_LIBRARIES}
)

