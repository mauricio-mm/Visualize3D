cmake_minimum_required(VERSION 3.20)

project(Visualize3D VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(raylib STATIC IMPORTED)
set_target_properties(raylib PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libraylib.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_executable(Visualize3D
    src/main.cpp
    src/environment3d.cpp
    src/denavit_hartenberg.cpp
    src/inverse_kinematics.cpp
    src/ui.cpp
)

target_include_directories(Visualize3D PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_options(Visualize3D PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wformat=2
    -Wcast-align
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Debug>:-O0>
)

target_link_libraries(Visualize3D PRIVATE
    raylib
    opengl32
    gdi32
    winmm
)
