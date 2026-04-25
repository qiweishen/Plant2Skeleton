set(Eigen_VERSION 3.4.0)

add_library(Eigen INTERFACE)

target_include_directories(Eigen SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR})
