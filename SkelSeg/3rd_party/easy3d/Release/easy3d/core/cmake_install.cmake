# Install script for directory: /Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core

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
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_core.2.5.4.dylib;/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_core.2.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_core.2.5.4.dylib"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_core.2.dylib"
    )
  foreach(file
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_core.2.5.4.dylib"
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_core.2.dylib"
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
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_core.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_core.dylib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/box.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/constant.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/curve.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/eigen_solver.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/graph.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/hash.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/heap.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/line.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/surface_mesh_builder.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/mat.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/matrix.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/matrix_algo.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/model.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/oriented_line.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/plane.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/point_cloud.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/principal_axes.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/property.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/quat.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/random.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/rect.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/segment.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/signal.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/spline_curve_fitting.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/spline_curve_interpolation.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/spline_interpolation.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/surface_mesh.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/poly_mesh.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/polygon.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/types.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core/vec.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/core" TYPE FILE FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/box.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/constant.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/curve.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/eigen_solver.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/graph.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/hash.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/heap.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/line.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/surface_mesh_builder.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/mat.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/matrix.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/matrix_algo.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/model.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/oriented_line.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/plane.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/point_cloud.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/principal_axes.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/property.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/quat.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/random.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/rect.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/segment.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/signal.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/spline_curve_fitting.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/spline_curve_interpolation.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/spline_interpolation.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/surface_mesh.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/poly_mesh.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/polygon.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/types.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/core/vec.h"
    )
endif()

