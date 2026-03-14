#include "../include/Game.h"
#include <iostream>
#include "ResourceManager.h"
#include "Entity.h"
#include <memory>
#include "FlapControllerComponent.h"
#include "GameManagerComponent.h"
#include "LinearMovementComponent.h"
#include "SceneLoader.h"
#include "ComponentRegistry.h"
#include "Engine.h"

Game::Game() {
}

Game::~Game() {
}

void Game::init(Engine& engine) {

    // Prime the Component Registry with Game-Specific Components
    ComponentRegistry::registerComponent("FlapControllerComponent", FlapControllerComponent::deserialize);
    ComponentRegistry::registerComponent("GameManagerComponent", GameManagerComponent::deserialize);
    
    ResourceManager::parseForceModelFile("assets/models/david.ForceModel");

    SceneLoader::loadScene("assets/scene.ForceScene", engine.rootEntity, engine.getWindow());

    // Find Camera and VisualPlayer
    for (auto& child : engine.rootEntity->children) {
        if (child->name == "Camera" && child->getComponent<CameraComponent>()) {
            engine.activeCamera = child->getComponent<CameraComponent>().get();
        }
        else if (child->name == "Player") {
            visualEntity = child->findChildByName("VisualPlayer");
        }
    }
}