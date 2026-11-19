cmake_minimum_required(VERSION 3.24)
project(ForceEngine)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Core Gui Widgets PATHS /opt/homebrew/opt/qtbase /opt/homebrew /usr/local/opt/qtbase)
find_package(Qt6Svg PATHS /opt/homebrew/opt/qtsvg/lib/cmake/Qt6Svg /opt/homebrew /usr/local)

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
set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(assimp)

# Fetch SDL_shadercross
set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "" FORCE)
FetchContent_Declare(
    SDL_shadercross
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_shadercross.git
    GIT_TAG        main
)
set(SDLSHADERCROSS_SHARED OFF CACHE BOOL "" FORCE)
set(SDLSHADERCROSS_STATIC ON CACHE BOOL "" FORCE)
set(SDLSHADERCROSS_VENDORED ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(SDL_shadercross)

# Common Engine and Subsystem Sources
set(COMMON_SOURCES
    src/Game.cpp
    src/Engine.cpp
    src/stb_image.cpp
    src/ResourceManager.cpp
    src/Renderer.cpp
    src/PhysicsSystem.cpp
    src/PhysicsComponent.cpp
    src/FirstPersonControllerComponent.cpp
    src/ProceduralWorldGenerator.cpp
    src/MapBaker.cpp
    src/MapLoader.cpp
    # UI System
    src/UI/UIRenderer.cpp
    # Road Editor & Simulation
    src/Road/GeoJsonLoader.cpp
    src/Road/RoadGenerator.cpp
    src/Road/RoadElevationSolver.cpp
    src/Road/RoadBinary.cpp
    src/Road/RoadBaker.cpp
    src/Road/RoadRenderer.cpp
    src/Road/VehiclePhysics.cpp
    src/Road/RoadEditor.cpp
    src/Road/RoadTerrainViewer.cpp
    # Terrain System
    src/Terrain/TerrainLoader.cpp
    src/Terrain/TerrainRenderer.cpp
    src/Terrain/QuadsphereLoader.cpp
    # New RHI Layer
    src/RHI/RHI_Device.cpp
    src/RHI/RHI_Buffer.cpp
    src/RHI/RHI_Texture.cpp
    src/RHI/RHI_Shader.cpp
    src/RHI/RHI_Pipeline.cpp
    src/RHI/RHI_ShaderWrapper.cpp
)

find_package(ZLIB REQUIRED)

# Zstandard (Zstd) compression library
find_path(ZSTD_INCLUDE_DIR NAMES zstd.h PATHS /opt/homebrew/include /usr/local/include /usr/include)
find_library(ZSTD_LIBRARY NAMES zstd PATHS /opt/homebrew/lib /opt/homebrew/opt/zstd/lib /usr/local/lib /usr/lib)
if (ZSTD_INCLUDE_DIR AND ZSTD_LIBRARY)
    message(STATUS "Found Zstd: ${ZSTD_LIBRARY} (include: ${ZSTD_INCLUDE_DIR})")
    add_library(Zstd::Zstd UNKNOWN IMPORTED)
    set_target_properties(Zstd::Zstd PROPERTIES
        IMPORTED_LOCATION "${ZSTD_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ZSTD_INCLUDE_DIR}"
    )
endif()

# LERC (Limited Error Raster Compression)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(external/lerc)

# 1. ForceEngine Master / Road Editor Executable
add_executable(${PROJECT_NAME} src/main.cpp ${COMMON_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE 
    include
    external/lerc/src/LercLib/include
    ${CMAKE_BINARY_DIR}/_deps/assimp-src/contrib/rapidjson/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp ZLIB::ZLIB Lerc Zstd::Zstd)

# 1b. ForceEngine Road Editor Qt Application
if (Qt6_FOUND)
    message(STATUS "Found Qt6: Building RoadEditorQt native Qt application")
    set(QT_ROAD_SOURCES
        include/Road/Qt/ChunkInputDialog.h
        include/Road/Qt/RoadViewportWidget.h
        include/Road/Qt/RoadEditorMainWindow.h
        src/Road/Qt/main_qt.cpp
        src/Road/Qt/RoadViewportWidget.cpp
        src/Road/Qt/ChunkInputDialog.cpp
        src/Road/Qt/RoadEditorMainWindow.cpp
    )
    add_executable(RoadEditorQt ${QT_ROAD_SOURCES} ${COMMON_SOURCES})
    target_include_directories(RoadEditorQt PRIVATE 
        include
        external/lerc/src/LercLib/include
        ${CMAKE_BINARY_DIR}/_deps/assimp-src/contrib/rapidjson/include
    )
    target_link_libraries(RoadEditorQt PRIVATE 
        Qt6::Core Qt6::Gui Qt6::Widgets
        SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp ZLIB::ZLIB Lerc Zstd::Zstd
    )
    if (TARGET Qt6::Svg)
        target_link_libraries(RoadEditorQt PRIVATE Qt6::Svg)
    endif()
endif()

# 2. Dedicated Road on Terrain Integration Viewer Executable
add_executable(RoadTerrainViewer src/road_terrain_main.cpp ${COMMON_SOURCES})
target_include_directories(RoadTerrainViewer PRIVATE 
    include
    external/lerc/src/LercLib/include
    ${CMAKE_BINARY_DIR}/_deps/assimp-src/contrib/rapidjson/include
)
target_link_libraries(RoadTerrainViewer PRIVATE SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp ZLIB::ZLIB Lerc Zstd::Zstd)

# 4. Standalone Road Baker CLI Executable
add_executable(RoadBaker src/road_baker_main.cpp ${COMMON_SOURCES})
target_include_directories(RoadBaker PRIVATE 
    include
    external/lerc/src/LercLib/include
    ${CMAKE_BINARY_DIR}/_deps/assimp-src/contrib/rapidjson/include
)
target_link_libraries(RoadBaker PRIVATE SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp ZLIB::ZLIB Lerc Zstd::Zstd)

# Topology & Serialization Unit Test
add_executable(test_topology
    tests/test_topology.cpp
    src/Terrain/TerrainLoader.cpp
    src/Road/GeoJsonLoader.cpp
    src/Road/RoadGenerator.cpp
    src/Road/RoadElevationSolver.cpp
    src/Road/RoadBinary.cpp
    src/RHI/RHI_Device.cpp
    src/RHI/RHI_Buffer.cpp
)
target_include_directories(test_topology PRIVATE 
    include
    external/lerc/src/LercLib/include
    ${CMAKE_BINARY_DIR}/_deps/assimp-src/contrib/rapidjson/include
)
target_link_libraries(test_topology PRIVATE SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp ZLIB::ZLIB Lerc Zstd::Zstd)

# Road on Terrain Integration Unit Test
add_executable(test_road_terrain
    tests/test_road_terrain.cpp
    src/Terrain/TerrainLoader.cpp
    src/Road/GeoJsonLoader.cpp
    src/Road/RoadGenerator.cpp
    src/Road/RoadElevationSolver.cpp
    src/Road/RoadBinary.cpp
    src/RHI/RHI_Device.cpp
    src/RHI/RHI_Buffer.cpp
)
target_include_directories(test_road_terrain PRIVATE 
    include
    external/lerc/src/LercLib/include
    ${CMAKE_BINARY_DIR}/_deps/assimp-src/contrib/rapidjson/include
)
target_link_libraries(test_road_terrain PRIVATE SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp ZLIB::ZLIB Lerc Zstd::Zstd)

# Terrain Loader & Curvature Math Unit Test
add_executable(test_terrain
    tests/test_terrain.cpp
    src/Terrain/TerrainLoader.cpp
)
target_include_directories(test_terrain PRIVATE 
    include
    external/lerc/src/LercLib/include
)
target_link_libraries(test_terrain PRIVATE ZLIB::ZLIB Lerc Zstd::Zstd)

# Quadsphere Math Unit Test (SPHERE.md Phase 1)
add_executable(test_quadsphere
    tests/test_quadsphere.cpp
    src/Terrain/QuadsphereLoader.cpp
)
target_include_directories(test_quadsphere PRIVATE 
    include
)
target_link_libraries(test_quadsphere PRIVATE Zstd::Zstd)

# Quadsphere Terrain Compiler CLI Tool (SPHERE.md Phase 2)
add_executable(compile_quadsphere
    src/Terrain/QuadsphereCompiler.cpp
    src/Terrain/QuadsphereLoader.cpp
    src/Terrain/TerrainLoader.cpp
)
target_include_directories(compile_quadsphere PRIVATE 
    include
    external/lerc/src/LercLib/include
)
target_link_libraries(compile_quadsphere PRIVATE Zstd::Zstd Lerc ZLIB::ZLIB)

# Quadsphere Renderer & Streaming Integration Test (SPHERE.md Phase 3)
add_executable(test_quadsphere_renderer
    tests/test_quadsphere_renderer.cpp
    ${COMMON_SOURCES}
)
target_include_directories(test_quadsphere_renderer PRIVATE 
    include
    external/lerc/src/LercLib/include
    ${CMAKE_BINARY_DIR}/_deps/assimp-src/contrib/rapidjson/include
)
target_link_libraries(test_quadsphere_renderer PRIVATE SDL3::SDL3-static SDL3_shadercross::SDL3_shadercross assimp ZLIB::ZLIB Lerc Zstd::Zstd)
