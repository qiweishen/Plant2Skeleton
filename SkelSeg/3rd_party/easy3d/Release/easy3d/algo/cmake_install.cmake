# Install script for directory: /Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local/Cellar/easy3d/2.5.4")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "lib" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_algo.2.5.4.dylib;/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_algo.2.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_algo.2.5.4.dylib"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_algo.2.dylib"
    )
  foreach(file
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_algo.2.5.4.dylib"
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_algo.2.dylib"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      execute_process(COMMAND /usr/bin/install_name_tool
        -delete_rpath "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib"
        -add_rpath "/usr/local/Cellar/easy3d/2.5.4/lib"
        "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/strip" -x "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "lib" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_algo.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_algo.dylib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/collider.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/delaunay.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/delaunay_2d.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/delaunay_3d.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/extrusion.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_geometry.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/gaussian_noise.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/point_cloud_normals.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/point_cloud_poisson_reconstruction.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/point_cloud_ransac.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/point_cloud_simplification.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/polygon_partition.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_components.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_curvature.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_enumerator.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_factory.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_fairing.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_features.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_geodesic.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_hole_filling.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_parameterization.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_polygonization.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_remeshing.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_sampler.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_simplification.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_smoothing.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_stitching.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_subdivision.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_tetrahedralization.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_topology.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/surface_mesh_triangulation.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/tessellator.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/text_mesher.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo/triangle_mesh_kdtree.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/algo" TYPE FILE FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/collider.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/delaunay.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/delaunay_2d.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/delaunay_3d.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/extrusion.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_geometry.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/gaussian_noise.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/point_cloud_normals.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/point_cloud_poisson_reconstruction.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/point_cloud_ransac.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/point_cloud_simplification.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/polygon_partition.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_components.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_curvature.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_enumerator.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_factory.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_fairing.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_features.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_geodesic.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_hole_filling.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_parameterization.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_polygonization.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_remeshing.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_sampler.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_simplification.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_smoothing.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_stitching.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_subdivision.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_tetrahedralization.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_topology.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/surface_mesh_triangulation.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/tessellator.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/text_mesher.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/algo/triangle_mesh_kdtree.h"
    )
endif()

