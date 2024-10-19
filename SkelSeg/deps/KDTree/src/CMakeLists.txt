set(KDTree_VERSION 2.0.1)

add_library(KDTree KDTree.cpp)

target_include_directories(KDTree PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

set_target_properties(KDTree PROPERTIES
        VERSION ${KDTree_VERSION}
)
