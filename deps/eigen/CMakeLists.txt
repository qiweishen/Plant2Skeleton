set(EIGEN_VERSION 3.4.0)

add_library(Eigen INTERFACE)

target_include_directories(Eigen INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(Eigen PROPERTIES
        EXPORT_NAME Eigen
        INTERFACE_EIGEN_VERSION ${EIGEN_VERSION}
)
