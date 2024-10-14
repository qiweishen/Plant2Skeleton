set(fast_int 0.1.0)

add_library(fast_int INTERFACE)

target_include_directories(fast_int INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(fast_int PROPERTIES
        INTERFACE_FAST_INT_VERSION ${fast_int}
)
