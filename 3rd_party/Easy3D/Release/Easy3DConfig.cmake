# ******************************************************************************
#      Copyright (C) 2015 Liangliang Nan <liangliang.nan@gmail.com>
#      https://3d.bk.tudelft.nl/liangliang/
#
#      This file is part of Easy3D. If it is useful in your research/work,
#      I would be grateful if you show your appreciation by citing it:
#      ------------------------------------------------------------------
#           Liangliang Nan.
#           Easy3D: a lightweight, easy-to-use, and efficient C++ library
#           for processing and rendering 3D data.
#           Journal of Open Source Software, 6(64), 3255, 2021.
#      ------------------------------------------------------------------
#
#      Easy3D is free software; you can redistribute it and/or modify
#      it under the terms of the GNU General Public License Version 3
#      as published by the Free Software Foundation.
#
#      Easy3D is distributed in the hope that it will be useful,
#      but WITHOUT ANY WARRANTY; without even the implied warranty of
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#      GNU General Public License for more details.
#
#      You should have received a copy of the GNU General Public License
#      along with this program. If not, see <http://www.gnu.org/licenses/>.
# ******************************************************************************

# Config file for Easy3D
# Reference:
#   How to create a ProjectConfig.cmake file
#   https://gitlab.kitware.com/cmake/community/-/wikis/doc/tutorials/How-to-create-a-ProjectConfig.cmake-file
#
# It defines the following variables:
#       Easy3D_FOUND                - True if headers and requested libraries were found.
#       Easy3D_INCLUDE_DIRS         - Easy3D include directories
#       Easy3D_LIBRARY_DIRS         - Link directories for Easy3D libraries.
#       Easy3D_AVAILABLE_COMPONENTS - All available (but may not be requested) Easy3D components.
#       Easy3D_<component>_FOUND    - True if ``<component>`` was found (``<component>`` name is lower-case).
#       Easy3D_VERSION_STRING       - Easy3D version number in ``X.Y.Z`` format.
#       Easy3D_VERSION              - Easy3D version number in ``X.Y.Z`` format (same as ``Easy3D_VERSION_STRING``)
#       Easy3D_CGAL_SUPPORT         - True if Easy3D was built with CGAL support.
#       Easy3D_FFMPEG_SUPPORT       - True if Easy3D was built with FFMPEG support.
#
# NOTE: The recommended way to specify libraries and headers with CMake is to use the target_link_libraries
#       command. This command automatically adds appropriate include directories, compile definitions, the
#       position-independent-code flag, and links to requested libraries.

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW) # support the use of "IN_LIST"
endif()

include(CMakeFindDependencyMacro)
find_dependency(Threads)

# Compute paths
get_filename_component(Easy3D_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set(Easy3D_INCLUDE_DIRS "/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D")
set(Easy3D_LIBRARY_DIRS "/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/lib")
set(Easy3D_AVAILABLE_COMPONENTS "util;core;fileio;kdtree;algo;renderer;gui;viewer")
set(Easy3D_VERSION_STRING "2.5.4")
set(Easy3D_VERSION 2.5.4)

set(found_components "")
set(missing_components "")
foreach(component ${Easy3D_FIND_COMPONENTS})
    string(TOLOWER ${component} component)  # not case sensitive
    if (component MATCHES "^easy3d::")      # treat 'easy3d::name' as 'name'
        string(REPLACE "easy3d::" "" component ${component})
    endif ()
    if (component MATCHES "^easy3d_")       # treat 'easy3d_name' as 'name'
        string(REPLACE "easy3d_" "" component ${component})
    endif ()

    if (component IN_LIST Easy3D_AVAILABLE_COMPONENTS)
        list(APPEND found_components ${component})
        set(Easy3D_${component}_FOUND TRUE)
    else ()
        list(APPEND missing_components ${component})
        set(Easy3D_${component}_FOUND FALSE)
    endif ()
endforeach()

set(LIB_TYPE SHARED)
string(TOLOWER "${LIB_TYPE} libraries" lib_type)
if (Easy3D_FIND_COMPONENTS) # user tries to find components
    set(components_to_link ${found_components})
    if (missing_components)
        message(WARNING "Not all requested Easy3D components have been found.\n"
                        "Found components (${lib_type}): ${found_components}\n"
                        "Missing components: ${missing_components}\n"
                        "Make sure all components have been built and their names are correctly specified. "
                        "The available Easy3D components (${lib_type}): ${Easy3D_AVAILABLE_COMPONENTS}\n")
    elseif (found_components)
        set(Easy3D_FOUND TRUE)
        message(STATUS "Found Easy3D (version '2.5.4'). Found components (${lib_type}): ${found_components}")
        message(STATUS "Easy3D_INCLUDE_DIRS: ${Easy3D_INCLUDE_DIRS}")
        message(STATUS "Easy3D_LIBRARY_DIRS: ${Easy3D_LIBRARY_DIRS}")
    endif()
else ()
    set(components_to_link ${Easy3D_AVAILABLE_COMPONENTS})
    foreach(component ${Easy3D_AVAILABLE_COMPONENTS})
        set(Easy3D_${component}_FOUND TRUE)
    endforeach()
    message(STATUS "Found Easy3D (version '2.5.4'). Available components (${lib_type}): ${Easy3D_AVAILABLE_COMPONENTS}")
    message(STATUS "Easy3D_INCLUDE_DIRS: ${Easy3D_INCLUDE_DIRS}")
    message(STATUS "Easy3D_LIBRARY_DIRS: ${Easy3D_LIBRARY_DIRS}")
endif ()

# Resolve dependencies
if ("algo_ext" IN_LIST components_to_link)
    set (Easy3D_CGAL_SUPPORT TRUE)
    message(STATUS "Easy3D_CGAL_SUPPORT: ${Easy3D_CGAL_SUPPORT} (Easy3D was built with CGAL-v)")
    if ("${LIB_TYPE}" STREQUAL "STATIC")
        find_dependency(CGAL) # needed only when the lib was STATIC
    endif()
endif()
if ("video" IN_LIST components_to_link)
    set (Easy3D_FFMPEG_SUPPORT TRUE)
    message(STATUS "Easy3D_FFMPEG_SUPPORT: ${Easy3D_FFMPEG_SUPPORT} (Easy3D was built with FFMPEG support)")
endif()

# Our library dependencies (contains definitions for IMPORTED targets)
include("${Easy3D_CMAKE_DIR}/Easy3DTargets.cmake")
