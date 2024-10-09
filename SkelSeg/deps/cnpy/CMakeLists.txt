add_library(cnpy STATIC
        cnpy.cpp
)

target_include_directories(cnpy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


list(APPEND THIRD_PARTY_LIBS lib1)
set(THIRD_PARTY_LIBS "${THIRD_PARTY_LIBS}" CACHE INTERNAL "List of third-party libraries")
