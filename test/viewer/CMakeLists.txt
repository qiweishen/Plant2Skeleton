cmake_minimum_required(VERSION 3.22...3.25)
project(Viewer_Test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Easy3D
set(easy3d_DIR "/Users/qiwei/Desktop/Easy3D/Release")
find_package(easy3d REQUIRED)
if(easy3d_FOUND)
    message(STATUS "Easy3D found: ${easy3d_VERSION}")
endif()

# Easy3D source root
set(EASY3D_ROOT "/Users/qiwei/Desktop/Easy3D")

# ImGui source files
set(IMGUI_DIR "${EASY3D_ROOT}/3rd_party/imgui")
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

set(SOURCES
    main.cpp
    viewer.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES} ${IMGUI_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${EASY3D_ROOT}/3rd_party/glfw/include
    ${EASY3D_ROOT}
)

# Link all required Easy3D components
target_link_libraries(${PROJECT_NAME} PUBLIC
    easy3d::viewer
    easy3d::core
    easy3d::fileio
    easy3d::util
    easy3d::renderer
)
