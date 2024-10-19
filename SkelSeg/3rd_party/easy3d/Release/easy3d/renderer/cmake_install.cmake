# Install script for directory: /Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer

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
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_renderer.2.5.4.dylib;/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_renderer.2.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_renderer.2.5.4.dylib"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_renderer.2.dylib"
    )
  foreach(file
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_renderer.2.5.4.dylib"
      "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_renderer.2.dylib"
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
   "/usr/local/Cellar/easy3d/2.5.4/lib/libeasy3d_renderer.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib" TYPE SHARED_LIBRARY FILES "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/Release/lib/libeasy3d_renderer.dylib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/ambient_occlusion.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/average_color_blending.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/camera.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/clipping_plane.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/constraint.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/drawable.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/drawable_lines.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/drawable_points.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/drawable_triangles.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/dual_depth_peeling.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/eye_dome_lighting.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/frame.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/framebuffer_object.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/frustum.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/key_frame_interpolator.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/manipulator.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/manipulated_camera_frame.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/manipulated_frame.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/opengl.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/opengl_error.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/opengl_util.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/opengl_timer.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/shape.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/read_pixel.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/buffer.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/renderer.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/shader_manager.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/shader_program.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/shadow.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/soft_shadow.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/state.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/texture.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/texture_manager.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/text_renderer.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/transform.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/transform_decompose.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/transparency.h;/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer/vertex_array_object.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/include/easy3d/renderer" TYPE FILE FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/ambient_occlusion.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/average_color_blending.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/camera.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/clipping_plane.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/constraint.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/drawable.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/drawable_lines.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/drawable_points.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/drawable_triangles.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/dual_depth_peeling.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/eye_dome_lighting.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/frame.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/framebuffer_object.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/frustum.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/key_frame_interpolator.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/manipulator.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/manipulated_camera_frame.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/manipulated_frame.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/opengl.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/opengl_error.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/opengl_util.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/opengl_timer.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/shape.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/read_pixel.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/buffer.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/renderer.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/shader_manager.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/shader_program.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/shadow.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/soft_shadow.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/state.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/texture.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/texture_manager.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/text_renderer.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/transform.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/transform_decompose.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/transparency.h"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/easy3d/renderer/vertex_array_object.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/include/3rd_party/glew/include/GL/glew.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/include/3rd_party/glew/include/GL" TYPE FILE FILES "/Users/shenqiwei/Desktop/Plant2Skeleton/Easy3D/3rd_party/glew/include/GL/glew.h")
endif()

