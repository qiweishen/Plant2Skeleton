set(fast_int_VERSION 0.1.0)

add_library(fast_int INTERFACE)

target_include_directories(fast_int SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
