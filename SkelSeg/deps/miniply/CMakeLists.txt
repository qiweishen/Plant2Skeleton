add_library(miniply STATIC
        miniply.cpp
)

target_include_directories(miniply PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


list(APPEND THIRD_PARTY_LIBS miniply)
set(THIRD_PARTY_LIBS "${THIRD_PARTY_LIBS}" CACHE INTERNAL "List of third-party libraries")
