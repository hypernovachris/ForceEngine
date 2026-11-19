#include "../include/Game.h"
#include <iostream>
#include "ResourceManager.h"
#include "Entity.h"
#include <memory>
#include "Engine.h"
#include "CameraComponent.h"
#include "FirstPersonControllerComponent.h"
#include "MapBaker.h"
#include "MapLoader.h"
#include "TriggerSystem.h"
#include <fstream>

Game::Game() {
}

Game::~Game() {
}

void Game::init(Engine& engine) {
    std::cout << "Initializing ForceEngine..." << std::endl;

    const std::string mapPath = "assets/maps/courtyard.fmap";

    // 1. Check if the compiled map exists; if not, bake it once
    std::ifstream mapCheck(mapPath, std::ios::binary);
    if (!mapCheck.is_open()) {
        std::cout << "Compiled map not found. Baking " << mapPath << "..." << std::endl;
        MapBaker::bakeProceduralCourtyard(mapPath);
    } else {
        mapCheck.close();
    }

    // 2. Load the compiled binary map (zero-copy GPU buffer upload)
    MapLoadResult mapInfo = MapLoader::loadMap(mapPath, engine);

    // 3. Register gameplay trigger callback
    TriggerSystem::getInstance().setCallback([](const FMapTrigger& trigger, bool isEnter) {
        if (isEnter) {
            std::cout << "\n>>> [TRIGGER ENTER] Zone: '" << trigger.tag 
                      << "' (Event ID: " << trigger.eventId << ") <<<" << std::endl;
        } else {
            std::cout << ">>> [TRIGGER EXIT]  Zone: '" << trigger.tag << "' <<<\n" << std::endl;
        }
    });

    // 4. Create Player with Camera and FirstPersonController at map spawn location
    auto player = std::make_shared<Entity>();
    player->name = "Player";
    player->position = mapInfo.success ? mapInfo.playerStart : glm::vec3(0.0f, 1.7f, 6.0f);

    auto camera = std::make_shared<CameraComponent>(70.0f, 800.0f / 600.0f, 0.1f, 300.0f);
    player->addComponent(camera);
    engine.activeCamera = camera.get();

    auto controller = std::make_shared<FirstPersonControllerComponent>(engine.getWindow());
    controller->moveSpeed = 6.5f;
    controller->sprintMultiplier = 1.6f;
    controller->shootSpeed = 22.0f;
    controller->ballRadius = 0.35f;
    controller->ballRestitution = 0.82f;
    if (mapInfo.success) {
        controller->yaw = mapInfo.playerStartYaw;
        controller->pitch = mapInfo.playerStartPitch;
    }
    player->addComponent(controller);

    engine.rootEntity->addChild(player);

    std::cout << "\n========================================================" << std::endl;
    std::cout << " FORCE ENGINE - 3D PROCEDURAL EXPLORATION & PHYSICS DEMO" << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << " Controls:" << std::endl;
    std::cout << "  - Mouse Look: Look around freely (yaw & pitch)" << std::endl;
    std::cout << "  - WASD: Move player (slides against walls)" << std::endl;
    std::cout << "  - Left Shift: Sprint" << std::endl;
    std::cout << "  - Space: Jump" << std::endl;
    std::cout << "  - Left Mouse Button: Shoot bouncing balls!" << std::endl;
    std::cout << "  - Key 'R': Clear all balls" << std::endl;
    std::cout << "  - Key 'ESC': Toggle cursor capture" << std::endl;
    std::cout << "  - Key 'F3': Toggle debug wireframe mode" << std::endl;
    std::cout << "========================================================\n" << std::endl;
}