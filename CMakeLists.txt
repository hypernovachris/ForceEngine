cmake_minimum_required(VERSION 3.24)
project(ForceEngine)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Automatically fetch and build SDL3 from source
include(FetchContent)

FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG        release-3.2.4
    OVERRIDE_FIND_PACKAGE
)
# Disable tests to save time
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
# Build SDL as a static library
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(SDL3)

# Fetch Assimp
FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG        v5.3.1
)
# Disable Assimp extras to speed up the build
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INJECT_DEBUG_POSTFIX OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ZLIB OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(assimp)

# Fetch SDL_shadercross
FetchContent_Declare(
    SDL_shadercross
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_shadercross.git
    GIT_TAG        main
)
set(SDLSHADERCROSS_SHARED OFF CACHE BOOL "" FORCE)
set(SDLSHADERCROSS_STATIC ON CACHE BOOL "" FORCE)
set(SDLSHADERCROSS_VENDORED ON CACHE BOOL "" FORCE)
set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(SDL_shadercross)

# Gather our source files
set(SOURCES
    src/main.cpp
    src/Game.cpp
    src/Engine.cpp
    src/stb_image.cpp
    src/ResourceManager.cpp
    src/Renderer.cpp
    # New RHI Layer
    src/RHI/RHI_Device.cpp
    src/RHI/RHI_Buffer.cpp
    src/RHI/RHI_Texture.cpp
    src/RHI/RHI_Shader.cpp
    src/RHI/RHI_Pipeline.cpp
    src/RHI/RHI_ShaderWrapper.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Tell the compiler where to find the headers
target_include_directories(${PROJECT_NAME} PRIVATE include)

# Link SDL3, SDL_shadercross, and Assimp to our executable
target_link_libraries(${PROJECT_NAME} PRIVATE SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp)