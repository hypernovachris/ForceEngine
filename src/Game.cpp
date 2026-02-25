#include "../include/Game.h"
#include <iostream>
#include "Material.h"
#include "Mesh.h"
#include "PrimitiveBuilder.h"
#include "ResourceManager.h"
#include "RendererComponent.h"
#include "LightComponent.h"
#include "Entity.h"
#include <memory>
#include "SpinComponent.h"
#include <cstdlib>
#include "PhysicsComponent.h"
#include "ColliderComponent.h"
#include "FlapControllerComponent.h"
#include "GameManagerComponent.h"
#include "LinearMovementComponent.h"
#include "SceneLoader.h"
#include "ComponentRegistry.h"

std::vector<ColliderComponent*> ColliderComponent::allColliders;

Game::Game() {
    // Camera starts 6 units back on the Z axis
}

Game::~Game() {
    // Clean up entities
    // Automatic with shared_ptr
    entities.clear();
}

void Game::init(GLFWwindow* window) {

    // 1. Prime the Component Registry
    ComponentRegistry::registerComponent("RendererComponent", RendererComponent::deserialize);
    ComponentRegistry::registerComponent("PhysicsComponent", PhysicsComponent::deserialize);
    ComponentRegistry::registerComponent("ColliderComponent", ColliderComponent::deserialize);
    ComponentRegistry::registerComponent("FlapControllerComponent", FlapControllerComponent::deserialize);
    ComponentRegistry::registerComponent("CameraComponent", CameraComponent::deserialize);
    ComponentRegistry::registerComponent("LightComponent", LightComponent::deserialize);
    ComponentRegistry::registerComponent("GameManagerComponent", GameManagerComponent::deserialize);
    
    auto root_entity = std::make_shared<Entity>();
    entities.push_back(root_entity);

    ResourceManager::parseForceModelFile("assets/models/david.ForceModel");

    SceneLoader::loadScene("assets/scene.ForceScene", root_entity, window);

    // Find Camera and VisualPlayer
    for (auto& child : root_entity->children) {
        if (child->name == "Camera") {
            activeCamera = child->getComponent<CameraComponent>().get();
        }
        else if (child->name == "Player") {
            visualEntity = child->findChildByName("VisualPlayer");
        }
    }

    // Initialize debug model
    auto debugMaterial = ResourceManager::loadForceMaterial("assets/materials/sun.ForceMaterial");
    debugCubeModel = PrimitiveBuilder::createCube(1.0f, 1.0f, 1.0f, true, true);
    if (!debugCubeModel->meshes.empty()) {
        debugCubeModel->materials[0] = debugMaterial;
    }
}

void Game::processInput(GLFWwindow* window) {
    
    static bool escPressedLastFrame = false;
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        if (!escPressedLastFrame) {
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

void Game::update() {
    // Update all game objects
    for (size_t i = 0; i < entities.size(); ) {
        if (entities[i]->pendingDestroy) {
            entities.erase(entities.begin() + i);
        } else {
            entities[i]->update(deltaTime);
            entities[i]->updateSelfAndChild();
            ++i;
        }
    }
}

// 2. Traverse the Scene Graph and pass the data down
void Game::render() {
    renderer.clear();
    
    // Setup scene global data
    if (activeCamera) {
        renderer.beginScene(activeCamera);
    }

    // Lambda for recursive drawing
    // We can't use a local lambda easily with recursion in C++ without std::function overhead or auto deduction tricks
    // So let's just make a member helper or a free function helper that takes the renderer
    
    // Let's use a standard implementation
    for (auto& rootNode : entities) {
        renderer.submitNode(rootNode);
    }

    renderer.endScene();

    if (debugMode) {
        renderer.renderDebug(debugCubeModel);
    }
}