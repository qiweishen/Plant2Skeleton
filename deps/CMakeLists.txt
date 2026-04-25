add_subdirectory(eigen)

add_subdirectory(fmt)
add_subdirectory(geometry-central)
add_subdirectory(KDTree)
add_subdirectory(nlohmann)
add_subdirectory(plywoot)

# Easy3D — built from source with all standard modules
set(Easy3D_BUILD_SHARED_LIBS        OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_TUTORIALS          OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_DOCUMENTATION      OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_TESTS              OFF CACHE BOOL "" FORCE)
set(Easy3D_BUILD_PYTHON_BINDINGS    OFF CACHE BOOL "" FORCE)
set(Easy3D_ENABLE_CGAL              OFF CACHE BOOL "" FORCE)
set(Easy3D_ENABLE_QT                ON CACHE BOOL "" FORCE)
set(Easy3D_ENABLE_FFMPEG            OFF CACHE BOOL "" FORCE)
add_subdirectory(Easy3D)
