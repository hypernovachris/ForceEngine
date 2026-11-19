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
#include "PhysicsSystem.h"
#include "TriggerSystem.h"
#include "FirstPersonControllerComponent.h"

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

    window = SDL_CreateWindow(title, width, height, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!window) {
        std::cerr << "Failed to create SDL window: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    // Ensure the cursor is visible and free by default for UI and editor tools
    SDL_ShowCursor();
    SDL_SetWindowRelativeMouseMode(window, false);

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
    ComponentRegistry::registerComponent("FirstPersonControllerComponent", FirstPersonControllerComponent::deserialize);

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

bool Engine::initWithNativeView(void* nativeView, int width, int height) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
        return false;
    }

    if (!SDL_ShaderCross_Init()) {
        std::cerr << "Failed to initialize SDL_shadercross: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    SDL_PropertiesID props = SDL_CreateProperties();
#ifdef __APPLE__
    SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_COCOA_VIEW_POINTER, nativeView);
    SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_METAL_BOOLEAN, true);
#elif defined(_WIN32)
    SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_WIN32_HWND_POINTER, nativeView);
#elif defined(__linux__)
    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_X11_WINDOW_NUMBER, (Sint64)(uintptr_t)nativeView);
#endif
    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, width > 0 ? width : 640);
    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, height > 0 ? height : 480);
    SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_BORDERLESS_BOOLEAN, true);
    SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_HIGH_PIXEL_DENSITY_BOOLEAN, true);

    window = SDL_CreateWindowWithProperties(props);
    SDL_DestroyProperties(props);

    if (!window) {
        std::cerr << "Failed to create SDL window from native view: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    SDL_ShowCursor();

    if (!RHI_Device::getInstance().init(window)) {
        std::cerr << "Failed to initialize RHI Device on native window" << std::endl;
        return false;
    }

    ComponentRegistry::registerComponent("RendererComponent", RendererComponent::deserialize);
    ComponentRegistry::registerComponent("PhysicsComponent", PhysicsComponent::deserialize);
    ComponentRegistry::registerComponent("ColliderComponent", ColliderComponent::deserialize);
    ComponentRegistry::registerComponent("CameraComponent", CameraComponent::deserialize);
    ComponentRegistry::registerComponent("LightComponent", LightComponent::deserialize);
    ComponentRegistry::registerComponent("FirstPersonControllerComponent", FirstPersonControllerComponent::deserialize);

    rootEntity = std::make_shared<Entity>();
    renderer.init();

    auto debugMaterial = ResourceManager::loadForceMaterial("assets/materials/sun.ForceMaterial");
    debugCubeModel = PrimitiveBuilder::createCube(1.0f, 1.0f, 1.0f, true, true);
    if (!debugCubeModel->meshes.empty()) {
        debugCubeModel->materials[0] = debugMaterial;
    }

    isRunning = true;
    lastFrame = SDL_GetTicks() / 1000.0f;
    return true;
}

void Engine::stepFrame(float dt) {
    deltaTime = dt;
    update();
    render();
}

void Engine::resize(int width, int height) {
    if (window && width > 0 && height > 0) {
        SDL_SetWindowSize(window, width, height);
    }
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
    PhysicsSystem::getInstance().clear();
    TriggerSystem::getInstance().clear();
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
            if (event.key.key == SDLK_ESCAPE && !onEvent) {
                // Toggle relative mouse mode only if default game
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
        if (onEvent) {
            onEvent(event);
        }
    }
}

void Engine::update() {
    if (onUpdate) {
        onUpdate(deltaTime);
        return;
    }

    // 1. Process component updates (controllers, inputs, spawners)
    for (size_t i = 0; i < rootEntity->children.size(); ++i) {
        if (!rootEntity->children[i]->pendingDestroy) {
            rootEntity->children[i]->update(deltaTime);
        }
    }

    // 2. Global physics simulation step (forces, collisions, sphere-sphere impulses, world boundaries)
    PhysicsSystem::getInstance().update(deltaTime);

    // 3. Gameplay trigger updates
    if (activeCamera && activeCamera->owner) {
        TriggerSystem::getInstance().update(activeCamera->owner->position);
    }

    // 4. Clean up pendingDestroy entities and update world transforms for rendering
    for (size_t i = 0; i < rootEntity->children.size(); ) {
        if (rootEntity->children[i]->pendingDestroy) {
            rootEntity->children.erase(rootEntity->children.begin() + i);
        } else {
            rootEntity->children[i]->updateSelfAndChild();
            ++i;
        }
    }
}

void Engine::render() {
    if (onRender) {
        onRender();
        return;
    }

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
