# Install script for directory: /Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio

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
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_fileio.2.5.4.dylib;/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_fileio.2.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_fileio.2.5.4.dylib"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_fileio.2.dylib"
    )
  foreach(file
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_fileio.2.5.4.dylib"
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_fileio.2.dylib"
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
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_fileio.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_fileio.dylib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/image_io.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/graph_io.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/ply_reader_writer.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/point_cloud_io.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/point_cloud_io_ptx.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/point_cloud_io_vg.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/surface_mesh_io.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/poly_mesh_io.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio/translator.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/fileio" TYPE FILE FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/image_io.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/graph_io.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/ply_reader_writer.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/point_cloud_io.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/point_cloud_io_ptx.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/point_cloud_io_vg.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/surface_mesh_io.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/poly_mesh_io.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/fileio/translator.h"
    )
endif()

