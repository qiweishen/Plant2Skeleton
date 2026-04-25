cmake_minimum_required(VERSION 3.22...3.25)
project(SkelSeg_Viewer LANGUAGES CXX)

# ImGui source files (from Easy3D source tree)
set(Easy3D_ROOT "${CMAKE_SOURCE_DIR}/deps/Easy3D")
set(IMGUI_DIR "${Easy3D_ROOT}/3rd_party/imgui")
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
    ${Easy3D_ROOT}/3rd_party/glfw/include
    ${Easy3D_ROOT}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    SkelSeg_lib
)
