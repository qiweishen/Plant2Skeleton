set(nlohmann_json_VERSION 3.11.3)

add_library(nlohmann INTERFACE)

target_include_directories(nlohmann INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(nlohmann PROPERTIES
        INTERFACE_nlohmann_VERSION ${nlohmann_json_VERSION}
)
