#include <iostream>
#include <cassert>
#include <SDL3/SDL.h>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "RHI/RHI_Device.h"
#include "Terrain/TerrainRenderer.h"
#include "Terrain/QuadsphereMath.h"

int main() {
    std::cout << "=== Testing Quadsphere Terrain Renderer & Metal Streaming ===" << std::endl;

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "Failed to init SDL: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("TestQuadsphere", 800, 600, SDL_WINDOW_HIDDEN);
    if (!window) {
        std::cerr << "Failed to create SDL window: " << SDL_GetError() << std::endl;
        return 1;
    }

    if (!RHI_Device::getInstance().init(window)) {
        std::cerr << "Failed to init RHI_Device!" << std::endl;
        return 1;
    }

    TerrainRenderer renderer;
    if (!renderer.init(SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM, SDL_GPU_TEXTUREFORMAT_D32_FLOAT)) {
        std::cerr << "Failed to init TerrainRenderer!" << std::endl;
        return 1;
    }
    std::cout << "[PASS] TerrainRenderer initialized static grid & pipeline." << std::endl;

    if (!renderer.loadPack("jughandle/somerset.fquadspherepack")) {
        std::cerr << "Failed to load jughandle/somerset.fquadspherepack!" << std::endl;
        return 1;
    }
    std::cout << "[PASS] Loaded jughandle/somerset.fquadspherepack (Tile count: "
              << renderer.getQuadspherePack().header.tileCount << ")." << std::endl;

    // Simulate camera at Somerset coordinates (40.592676, -74.515211)
    glm::dvec3 camPosECEF = Quadsphere::geodeticToECEF(40.592676, -74.515211, 150.0);
    glm::vec3 camPos(0.0f, 150.0f, 0.0f);
    glm::vec3 camForward(0.0f, -0.3f, -0.95f);
    glm::mat4 view = glm::lookAt(camPos, camPos + camForward, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 proj = glm::perspective(glm::radians(60.0f), 800.0f / 600.0f, 0.5f, 25000.0f);

    // Update 5 frames to stream and upload tiles
    for (int frame = 0; frame < 5; ++frame) {
        renderer.update(camPos, camForward, view, proj, 800.0f / 600.0f, 0.016f, &camPosECEF);
    }

    std::cout << "Visible Tiles: " << renderer.getVisibleTiles().size() << std::endl;
    std::cout << "Rendered Tiles: " << renderer.getRenderedTileCount() << std::endl;
    std::cout << "Cached GPU Textures: " << renderer.getCachedTileCount() << std::endl;

    const auto& counts = renderer.getLodCounts();
    for (size_t i = 0; i < 9 && i < counts.size(); ++i) {
        if (counts[i] > 0) {
            std::cout << "  LOD " << (10 + i) << ": " << counts[i] << " tiles" << std::endl;
        }
    }

    assert(renderer.getRenderedTileCount() > 0 && "Expected visible rendered tiles!");
    std::cout << "[PASS] Quadsphere terrain quadtree streamed & rendered tiles successfully!" << std::endl;

    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "\nALL QUADSPHERE RENDERER TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
