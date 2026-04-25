add_library(KDTree src/KDTree.cpp)

target_include_directories(KDTree SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
