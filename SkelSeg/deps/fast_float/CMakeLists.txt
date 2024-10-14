set(fast_float_VERSION 6.1.6)

add_library(fast_float INTERFACE)

target_include_directories(fast_float INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(fast_float PROPERTIES
        EXPORT_NAME fast_float
        INTERFACE_fast_float_VERSION ${fast_float_VERSION}
)
