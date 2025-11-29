cmake_minimum_required(VERSION 3.20)
project(Aegis VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
include(FetchContent)

# 1. GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
FetchContent_MakeAvailable(glfw)

# 2. GLM (Math)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        0.9.9.8
)
FetchContent_MakeAvailable(glm)

# 3. Dear ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.89.9
)
FetchContent_MakeAvailable(imgui)

# --- Sources ---
file(GLOB_RECURSE SOURCES 
    "src/*.cpp" 
    "src/*.h"
    "include/*.h"
)

# ImGui Sources (Need to include implementation files)
set(IMGUI_DIR ${imgui_SOURCE_DIR})
list(APPEND SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# --- Executable ---
add_executable(Aegis ${SOURCES})

# --- Include Directories ---
target_include_directories(Aegis PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# --- Linking ---
target_link_libraries(Aegis PRIVATE
    glfw
    glm::glm
    ws2_32  # Winsock for Windows
)

# --- Compile Definitions ---
# Use OpenGL 3 for ImGui
target_compile_definitions(Aegis PRIVATE IMGUI_IMPL_OPENGL_LOADER_CUSTOM)
