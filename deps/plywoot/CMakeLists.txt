set(plywoot_VERSION 3.11.3)

add_subdirectory(deps)

add_library(plywoot INTERFACE)

target_include_directories(plywoot INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(plywoot INTERFACE
        fast_float
        fast_int
)

target_compile_definitions(plywoot INTERFACE
        PLYWOOT_USE_FAST_FLOAT
        PLYWOOT_USE_FAST_INT
)

set_target_properties(plywoot PROPERTIES
        INTERFACE_plywoot_VERSION ${plywoot_VERSION}
)
