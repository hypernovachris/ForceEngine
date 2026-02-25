#ifndef GAME_MANAGER_COMPONENT_H
#define GAME_MANAGER_COMPONENT_H

#include "Component.h"
#include "ColliderComponent.h"
#include "Entity.h"
#include "RendererComponent.h"
#include "LinearMovementComponent.h"
#include "PipeComponent.h"
#include "PhysicsComponent.h"
#include <iostream>
#include <memory>
#include <cstdlib>

class GameManagerComponent : public Component {
public:
    ColliderComponent* playerCollider = nullptr;
    std::shared_ptr<Model> pipeModel;
    std::shared_ptr<Entity> rootEntity;
    ColliderComponent* leftBoundary = nullptr;
    
    std::string playerEntityName;
    std::string leftBoundaryEntityName;

    float spawnTimer = 0.0f;
    float spawnInterval = 1.5f;

    GameManagerComponent() {}

    GameManagerComponent(ColliderComponent* playerCol, std::shared_ptr<Model> model, std::shared_ptr<Entity> root, ColliderComponent* leftBound) 
        : playerCollider(playerCol), pipeModel(model), rootEntity(root), leftBoundary(leftBound) {}

    void awake() override {
        // Resolve references if they are not set directly
        if (!rootEntity && owner) {
            // Find the root entity by going up
            Entity* current = owner;
            while (current->parent) {
                current = current->parent;
            }
            rootEntity = current->shared_from_this();
        }

        if (rootEntity) {
            if (!playerCollider && !playerEntityName.empty()) {
                auto playerEnt = rootEntity->findChildByName(playerEntityName);
                if (playerEnt) {
                    playerCollider = playerEnt->getComponent<ColliderComponent>().get();
                }
            }
            if (!leftBoundary && !leftBoundaryEntityName.empty()) {
                auto boundsEnt = rootEntity->findChildByName(leftBoundaryEntityName);
                if (boundsEnt) {
                    leftBoundary = boundsEnt->getComponent<ColliderComponent>().get();
                }
            }
        }
    }

    void resetGame() {
        if (playerCollider && playerCollider->owner) {
            playerCollider->owner->position = glm::vec3(-2.0f, 0.0f, 0.0f);
            auto physics = playerCollider->owner->getComponent<PhysicsComponent>();
            if (physics) {
                physics->velocity = glm::vec3(0.0f);
            }
        }
        if (rootEntity) {
            for (auto& child : rootEntity->children) {
                if (child->getComponent<PipeComponent>()) {
                    child->pendingDestroy = true;
                }
            }
        }
        spawnTimer = spawnInterval;
    }

    void update(float deltaTime) override {
        if (!playerCollider || !leftBoundary || !rootEntity || !pipeModel) return;

        // Loop through every collider in the game
        for (auto* otherCollider : ColliderComponent::allColliders) {
            
            // Don't let the bird collide with itself or the left boundary
            if (otherCollider == playerCollider || otherCollider == leftBoundary) continue;

            // Check for the overlap
            if (playerCollider->isCollidingWith(otherCollider)) {
                
                // GAME OVER STATE TRIGGERED
                resetGame();
                return;
            }
        }
        
        // Floor Collision Check (Hardcoded floor at Y = -5.0f for example)
        if (playerCollider->owner->position.y < -5.0f) {
            resetGame();
            return;
        }
        
        // Spawning logic
        spawnTimer -= deltaTime;
        if (spawnTimer <= 0.0f) {
            spawnTimer = spawnInterval;
            spawnPipes();
        }
    }
    
    void spawnPipes() {
        if (!rootEntity || !pipeModel || !leftBoundary) return;
        
        float xPos = 10.0f; // Spawn off screen to the right
        float gapCenter = -1.5f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/3.0f));
        float gapSize = 3.5f;

        // Bottom pipe
        auto bottomPipe = std::make_shared<Entity>();
        bottomPipe->position = glm::vec3(xPos, gapCenter - gapSize/2.0f - 5.0f, 0.0f); 
        bottomPipe->addComponent(std::make_shared<RendererComponent>(pipeModel));
        auto bottomCollider = std::make_shared<ColliderComponent>(glm::vec3(1.0f, 10.0f, 1.0f));
        bottomPipe->addComponent(bottomCollider);
        bottomPipe->addComponent(std::make_shared<LinearMovementComponent>(glm::vec3(-3.0f, 0.0f, 0.0f)));
        bottomPipe->addComponent(std::make_shared<PipeComponent>(bottomCollider.get(), leftBoundary));
        rootEntity->addChild(bottomPipe);

        // Top pipe
        auto topPipe = std::make_shared<Entity>();
        topPipe->position = glm::vec3(xPos, gapCenter + gapSize/2.0f + 5.0f, 0.0f);
        topPipe->addComponent(std::make_shared<RendererComponent>(pipeModel));
        auto topCollider = std::make_shared<ColliderComponent>(glm::vec3(1.0f, 10.0f, 1.0f));
        topPipe->addComponent(topCollider);
        topPipe->addComponent(std::make_shared<LinearMovementComponent>(glm::vec3(-3.0f, 0.0f, 0.0f)));
        topPipe->addComponent(std::make_shared<PipeComponent>(topCollider.get(), leftBoundary));
        rootEntity->addChild(topPipe);
    }

    static std::shared_ptr<Component> deserialize(std::istringstream& iss, GLFWwindow* window) {
        auto gm = std::make_shared<GameManagerComponent>();
        iss >> gm->playerEntityName >> gm->leftBoundaryEntityName;
        // The pipe model needs to be created or fetched from ResourceManager
        // We'll hardcode the fetching logic here or use PrimitiveBuilder
        auto pipeModel = PrimitiveBuilder::createCube(1.0f, 10.0f, 1.0f, true, true);
        auto pipeMaterial = ResourceManager::loadForceMaterial("assets/materials/glossy_tile.ForceMaterial");
        if (!pipeModel->meshes.empty()) {
            pipeModel->materials[0] = pipeMaterial;
        }
        gm->pipeModel = pipeModel;
        return gm;
    }
};

#endif