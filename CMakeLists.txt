cmake_minimum_required(VERSION 3.21)

# ==============================================================================
# Project Setup
# ==============================================================================
project(BasePhysics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# ==============================================================================
# External Dependencies
# ==============================================================================
set(SFML_ROOT "C:/Dev/SFML-2.6.2")
set(SFML_DIR "${SFML_ROOT}/lib/cmake/SFML")
find_package(SFML 2.6 COMPONENTS system window graphics audio network REQUIRED)

set(IMGUI_DIR "C:/Dev/imgui-1.90.9")
set(IMGUI_SOURCES 
    ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp 
    ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp)
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC ${IMGUI_DIR})

set(IMGUI_SFML_DIR "C:/Dev/imgui-sfml-2.6.1")
add_library(imgui-sfml STATIC ${IMGUI_SFML_DIR}/imgui-SFML.cpp)
target_include_directories(imgui-sfml PUBLIC ${IMGUI_SFML_DIR})
target_link_libraries(imgui-sfml PUBLIC imgui sfml-graphics sfml-window sfml-system)

# ==============================================================================
# BasePhysics Engine Library
# ==============================================================================
file(GLOB_RECURSE ENGINE_SRC "src/*.cpp")

add_library(BasePhysics STATIC ${ENGINE_SRC})

target_include_directories(BasePhysics PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
)

# ==============================================================================
# Demo Applications Executable
# ==============================================================================
file(GLOB_RECURSE DEMO_SRC "examples/*.cpp")

add_executable(BasePhysicsDemos ${DEMO_SRC})

target_link_libraries(BasePhysicsDemos 
    PRIVATE 
        BasePhysics
        imgui-sfml
        sfml-graphics
        sfml-window
        sfml-system
        sfml-audio
        sfml-network
)

if(WIN32)
    target_link_libraries(BasePhysicsDemos PRIVATE opengl32 winmm gdi32)
endif()