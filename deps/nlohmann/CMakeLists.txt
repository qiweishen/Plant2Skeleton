add_library(nlohmann INTERFACE)

target_include_directories(nlohmann SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR})
