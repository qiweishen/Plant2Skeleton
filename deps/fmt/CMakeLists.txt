set(FMT_SOURCES src/format.cc)
set(FMT_OS ON CACHE BOOL "Include OS-specific APIs.")

add_library(fmt STATIC ${FMT_SOURCES})
add_library(fmt::fmt ALIAS fmt)

if (FMT_OS)
  target_sources(fmt PRIVATE src/os.cc)
else()
  target_compile_definitions(fmt PRIVATE FMT_OS=0)
endif()

if (cxx_std_11 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
  target_compile_features(fmt PUBLIC cxx_std_11)
endif()

target_include_directories(fmt SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if (BUILD_SHARED_LIBS)
  target_compile_definitions(fmt PRIVATE FMT_LIB_EXPORT INTERFACE FMT_SHARED)
endif()
