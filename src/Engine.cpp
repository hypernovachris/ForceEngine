#include "Engine.h"
#include <iostream>
#include "ComponentRegistry.h"

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
    // 1. Initialize GLFW and Window
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);

    // Hide the cursor and capture it for the 3D camera
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // 2. Load GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return false;
    }
    glEnable(GL_DEPTH_TEST); // Enable 3D depth testing

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

    return true;
}

void Engine::run() {
    // 4. The Master Engine Loop
    while (!glfwWindowShouldClose(window)) {
        // Calculate deltaTime
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Execute Engine Stages
        processInput();
        update();
        render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void Engine::shutdown() {
    if (rootEntity) {
        rootEntity->children.clear();
        rootEntity.reset();
    }
    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
}

void Engine::processInput() {
    static bool escPressedLastFrame = false;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        if (!escPressedLastFrame) {
            // Toggle cursor capture
            int currentMode = glfwGetInputMode(window, GLFW_CURSOR);
            if (currentMode == GLFW_CURSOR_DISABLED) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            } else {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            }
            escPressedLastFrame = true;
        }
    } else {
        escPressedLastFrame = false;
    }

    static bool f3PressedLastFrame = false;
    if (glfwGetKey(window, GLFW_KEY_F3) == GLFW_PRESS) {
        if (!f3PressedLastFrame) {
            debugMode = !debugMode;
            f3PressedLastFrame = true;
        }
    } else {
        f3PressedLastFrame = false;
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
    renderer.clear();
    
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
