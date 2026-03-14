#include "Engine.h"
#include <iostream>
#include "ComponentRegistry.h"
#include <SDL3_shadercross/SDL_shadercross.h>

// Core components for priming the registry natively
#include "RendererComponent.h"
#include "PhysicsComponent.h"
#include "ColliderComponent.h"
#include "CameraComponent.h"
#include "LightComponent.h"
#include "PrimitiveBuilder.h"

std::vector<ColliderComponent*> ColliderComponent::allColliders;

Engine::Engine() {
}

Engine::~Engine() {
    shutdown();
}

bool Engine::init(int width, int height, const char* title) {
    // 1. Initialize SDL and Window
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
        return false;
    }

    if (!SDL_ShaderCross_Init()) {
        std::cerr << "Failed to initialize SDL_shadercross: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    window = SDL_CreateWindow(title, width, height, SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::cerr << "Failed to create SDL window: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    // Hide the cursor and capture it for the 3D camera
    SDL_HideCursor();
    SDL_SetWindowRelativeMouseMode(window, true);

    // 2. Initialize RHI Device
    if (!RHI_Device::getInstance().init(window)) {
        std::cerr << "Failed to initialize RHI Device" << std::endl;
        return false;
    }

    // 3. Prime the internal Component Registry with core engine components
    ComponentRegistry::registerComponent("RendererComponent", RendererComponent::deserialize);
    ComponentRegistry::registerComponent("PhysicsComponent", PhysicsComponent::deserialize);
    ComponentRegistry::registerComponent("ColliderComponent", ColliderComponent::deserialize);
    ComponentRegistry::registerComponent("CameraComponent", CameraComponent::deserialize);
    ComponentRegistry::registerComponent("LightComponent", LightComponent::deserialize);

    // Setup Engine Context
    rootEntity = std::make_shared<Entity>();

    // Setup Renderer
    renderer.init();

    // Initialize debug model mapping
    auto debugMaterial = ResourceManager::loadForceMaterial("assets/materials/sun.ForceMaterial");
    debugCubeModel = PrimitiveBuilder::createCube(1.0f, 1.0f, 1.0f, true, true);
    if (!debugCubeModel->meshes.empty()) {
        debugCubeModel->materials[0] = debugMaterial;
    }

    isRunning = true;
    lastFrame = SDL_GetTicks() / 1000.0f; // SDL_GetTicks is in milliseconds

    return true;
}

void Engine::run() {
    // 4. The Master Engine Loop
    while (isRunning) {
        // Calculate deltaTime
        float currentFrame = SDL_GetTicks() / 1000.0f;
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Execute Engine Stages
        processInput();
        update();
        render();
    }
}

void Engine::shutdown() {
    if (rootEntity) {
        rootEntity->children.clear();
        rootEntity.reset();
    }
    RHI_Device::getInstance().shutdown();
    if (window) {
        SDL_DestroyWindow(window);
        window = nullptr;
    }
    SDL_ShaderCross_Quit();
    SDL_Quit();
}

void Engine::processInput() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_EVENT_QUIT) {
            isRunning = false;
        } else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
            if (event.key.key == SDLK_ESCAPE) {
                // Toggle relative mouse mode
                bool relative = SDL_GetWindowRelativeMouseMode(window);
                SDL_SetWindowRelativeMouseMode(window, !relative);
                if (relative) {
                    SDL_ShowCursor();
                } else {
                    SDL_HideCursor();
                }
            } else if (event.key.key == SDLK_F3) {
                debugMode = !debugMode;
            }
        }
    }
}

void Engine::update() {
    // Traverse entity graph starting at root
    // Handle entity destruction safely, though normally we might use a dedicated system. 
    // rootEntity shouldn't be destroyed, so we just run update on it.
    
    // We update all children of root, and clean up those marked as pendingDestroy.
    for (size_t i = 0; i < rootEntity->children.size(); ) {
        if (rootEntity->children[i]->pendingDestroy) {
            rootEntity->children.erase(rootEntity->children.begin() + i);
        } else {
            rootEntity->children[i]->update(deltaTime);
            rootEntity->children[i]->updateSelfAndChild();
            ++i;
        }
    }
}

void Engine::render() {
    // Setup scene global data
    if (activeCamera) {
        renderer.beginScene(activeCamera);
    }

    // Pass the root down to draw all entities.
    renderer.submitNode(rootEntity);

    renderer.endScene();

    if (debugMode) {
        renderer.renderDebug(debugCubeModel);
    }
}
