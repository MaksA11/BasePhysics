cmake_minimum_required(VERSION 3.21)

project(BasePhysics LANGUAGES CXX)

# ============================================================
# C++ standard
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Windows specifics
# ============================================================
if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# ============================================================
# SFML 2.6.2
# ============================================================
set(SFML_ROOT "C:/Dev/SFML-2.6.2")
set(SFML_DIR "${SFML_ROOT}/lib/cmake/SFML")

find_package(SFML 2.6 COMPONENTS system window graphics audio network REQUIRED)

# ============================================================
# ImGui 1.90.9
# ============================================================
set(IMGUI_DIR "C:/Dev/imgui-1.90.9")

set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
)

add_library(imgui STATIC ${IMGUI_SOURCES})

target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
)

# ============================================================
# ImGui-SFML 2.6.1
# ============================================================
set(IMGUI_SFML_DIR "C:/Dev/imgui-sfml-2.6.1")

set(IMGUI_SFML_SOURCES
    ${IMGUI_SFML_DIR}/imgui-SFML.cpp
)

add_library(imgui-sfml STATIC ${IMGUI_SFML_SOURCES})

target_include_directories(imgui-sfml PUBLIC
    ${IMGUI_SFML_DIR}
)

target_link_libraries(imgui-sfml
    PUBLIC
        imgui
        sfml-graphics
        sfml-window
        sfml-system
)

# ============================================================
# My sources
# ============================================================
file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(BasePhysics ${SRC_FILES})

target_include_directories(BasePhysics PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(BasePhysics
    imgui-sfml
    imgui
    sfml-graphics
    sfml-window
    sfml-system
    sfml-audio
    sfml-network
)

# ============================================================
# Windows system libraries
# ============================================================
if(WIN32)
    target_link_libraries(BasePhysics
        opengl32
        winmm
        gdi32
    )
endif()
