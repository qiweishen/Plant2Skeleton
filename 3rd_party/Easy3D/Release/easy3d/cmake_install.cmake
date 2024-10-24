# Install script for directory: /Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/easy3d

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DConfig.cmake;/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DConfigVersion.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D" TYPE FILE FILES
    "/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/CMakeFiles/Easy3DConfig.cmake"
    "/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/Easy3DConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DTargets.cmake"
         "/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/CMakeFiles/Export/7a15dedd49bf174dae65d3c38f41ed7b/Easy3DTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DTargets.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D" TYPE FILE FILES "/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/CMakeFiles/Export/7a15dedd49bf174dae65d3c38f41ed7b/Easy3DTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D/Easy3DTargets-release.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
      message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
      message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    file(INSTALL DESTINATION "/usr/local/Cellar/easy3d/2.5.4/lib/CMake/Easy3D" TYPE FILE FILES "/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/CMakeFiles/Export/7a15dedd49bf174dae65d3c38f41ed7b/Easy3DTargets-release.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/util/cmake_install.cmake")
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/core/cmake_install.cmake")
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/fileio/cmake_install.cmake")
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/kdtree/cmake_install.cmake")
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/algo/cmake_install.cmake")
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/renderer/cmake_install.cmake")
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/gui/cmake_install.cmake")
  include("/Users/shenqiwei/Desktop/Plant2Skeleton/3rd_party/Easy3D/Release/easy3d/viewer/cmake_install.cmake")

endif()

