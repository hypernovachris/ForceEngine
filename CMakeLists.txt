cmake_minimum_required(VERSION 3.14)
project(ForceEngine)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Automatically fetch and build GLFW from source
include(FetchContent)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
# Disable GLFW's test and example builds to save time
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

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
set(ASSIMP_BUILD_ZLIB ON CACHE BOOL "" FORCE) 

FetchContent_MakeAvailable(assimp)

# 2. Gather our source files
set(SOURCES
    src/main.cpp
    src/Game.cpp
    src/glad.c
    src/Shader.cpp
    src/stb_image.cpp
    src/ResourceManager.cpp
    src/Renderer.cpp
)

# 3. Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})


# 4. Tell the compiler where to find the headers (GLAD and GLM)
target_include_directories(${PROJECT_NAME} PRIVATE include)

# 5. Link the GLFW library, Assimp and OpenGL to our executable
target_link_libraries(${PROJECT_NAME} PRIVATE glfw assimp ${OPENGL_LIBRARIES})