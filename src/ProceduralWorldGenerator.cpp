#include "ProceduralWorldGenerator.h"
#include "ResourceManager.h"
#include "PrimitiveBuilder.h"
#include "RendererComponent.h"
#include "PhysicsComponent.h"
#include "PhysicsSystem.h"
#include "LightComponent.h"
#include <iostream>
#include <cmath>

static uint32_t hashCoords(int x, int z, unsigned int seed) {
    uint32_t h = seed ^ (x * 73856093) ^ (z * 19349663);
    h = (h ^ (h >> 13)) * 0x5bd1e995;
    h ^= (h >> 15);
    return h;
}

void ProceduralWorldGenerator::generate(std::shared_ptr<Entity> rootEntity, const WorldConfig& config) {
    if (!rootEntity) return;

    std::cout << "ProceduralWorldGenerator: Generating " << config.gridSize << "x" << config.gridSize << " world..." << std::endl;

    // 1. Setup Materials
    auto floorMat = ResourceManager::loadForceMaterial("assets/materials/glossy_tile.ForceMaterial");
    if (floorMat) {
        floorMat->textureScale = glm::vec2(config.gridSize * 0.5f, config.gridSize * 0.5f);
    }
    auto wallMat = ResourceManager::loadForceMaterial("assets/materials/tung.ForceMaterial");
    auto pillarMat = ResourceManager::loadForceMaterial("assets/materials/david.ForceMaterial");
    auto sunMat = ResourceManager::loadForceMaterial("assets/materials/sun.ForceMaterial");
    if (sunMat) {
        sunMat->castsShadow = false;
    }

    // 2. Base Geometry
    auto unitCubeModel = PrimitiveBuilder::createCube(1.0f, 1.0f, 1.0f, true, true);
    auto cubeMesh = unitCubeModel->meshes[0];

    auto floorModel = std::make_shared<Model>();
    floorModel->meshes.push_back(cubeMesh);
    floorModel->materials.push_back(floorMat ? floorMat : wallMat);

    auto wallModel = std::make_shared<Model>();
    wallModel->meshes.push_back(cubeMesh);
    wallModel->materials.push_back(wallMat);

    auto pillarModel = std::make_shared<Model>();
    pillarModel->meshes.push_back(cubeMesh);
    pillarModel->materials.push_back(pillarMat ? pillarMat : wallMat);

    auto orbSphere = PrimitiveBuilder::createSphere(12, 12, 0.25f, true, true);
    auto orbModel = std::make_shared<Model>();
    orbModel->meshes.push_back(orbSphere->meshes[0]);
    orbModel->materials.push_back(sunMat);

    float totalSize = config.gridSize * config.cellSize;
    float halfSize = totalSize * 0.5f;

    // Set Physics Floor and World Bounds
    PhysicsSystem::getInstance().setFloorLevel(0.0f);
    PhysicsSystem::getInstance().setWorldBounds(-halfSize + 0.5f, halfSize - 0.5f, -halfSize + 0.5f, halfSize - 0.5f);

    // 3. Create Ground Floor
    auto floorEntity = std::make_shared<Entity>();
    floorEntity->name = "Floor";
    floorEntity->position = glm::vec3(0.0f, -0.2f, 0.0f);
    floorEntity->scale = glm::vec3(totalSize + 2.0f, 0.4f, totalSize + 2.0f);
    floorEntity->addComponent(std::make_shared<RendererComponent>(floorModel));
    rootEntity->addChild(floorEntity);

    // 4. Perimeter Walls
    float wallThickness = 1.2f;
    float wallH = config.wallHeight;

    auto makePerimeterWall = [&](const std::string& name, glm::vec3 pos, glm::vec3 scale) {
        auto wall = std::make_shared<Entity>();
        wall->name = name;
        wall->position = pos;
        wall->scale = scale;
        wall->addComponent(std::make_shared<RendererComponent>(wallModel));

        auto phys = std::make_shared<PhysicsComponent>();
        phys->setBox(scale * 0.5f);
        phys->setStatic(true);
        wall->addComponent(phys);

        rootEntity->addChild(wall);
    };

    // North Wall
    makePerimeterWall("Wall_North", glm::vec3(0.0f, wallH * 0.5f, -halfSize), glm::vec3(totalSize + wallThickness * 2.0f, wallH, wallThickness));
    // South Wall
    makePerimeterWall("Wall_South", glm::vec3(0.0f, wallH * 0.5f, halfSize), glm::vec3(totalSize + wallThickness * 2.0f, wallH, wallThickness));
    // West Wall
    makePerimeterWall("Wall_West", glm::vec3(-halfSize, wallH * 0.5f, 0.0f), glm::vec3(wallThickness, wallH, totalSize));
    // East Wall
    makePerimeterWall("Wall_East", glm::vec3(halfSize, wallH * 0.5f, 0.0f), glm::vec3(wallThickness, wallH, totalSize));

    // 5. Central Courtyard Low Raised Platform (Physics Bouncer in Center)
    {
        auto platform = std::make_shared<Entity>();
        platform->name = "Plaza_Platform";
        platform->position = glm::vec3(0.0f, 0.25f, 0.0f);
        platform->scale = glm::vec3(4.0f, 0.5f, 4.0f);
        platform->addComponent(std::make_shared<RendererComponent>(pillarModel));

        auto phys = std::make_shared<PhysicsComponent>();
        phys->setBox(glm::vec3(2.0f, 0.25f, 2.0f));
        phys->setStatic(true);
        platform->addComponent(phys);

        rootEntity->addChild(platform);
    }

    // 6. Courtyard Framing Columns
    float cDist = config.courtyardRadius * config.cellSize;
    glm::vec2 corners[4] = { {-cDist, -cDist}, {cDist, -cDist}, {-cDist, cDist}, {cDist, cDist} };
    for (int i = 0; i < 4; ++i) {
        auto pillar = std::make_shared<Entity>();
        pillar->name = "Courtyard_Column_" + std::to_string(i);
        pillar->position = glm::vec3(corners[i].x, config.wallHeight * 0.5f, corners[i].y);
        pillar->scale = glm::vec3(1.2f, config.wallHeight, 1.2f);
        pillar->addComponent(std::make_shared<RendererComponent>(pillarModel));

        auto phys = std::make_shared<PhysicsComponent>();
        phys->setBox(pillar->scale * 0.5f);
        phys->setStatic(true);
        pillar->addComponent(phys);

        rootEntity->addChild(pillar);
    }

    // 7. Grid Obstacles (Outside Courtyard)
    int radius = config.gridSize / 2;
    int obstacleCount = 0;

    for (int gx = -radius + 1; gx < radius; ++gx) {
        for (int gz = -radius + 1; gz < radius; ++gz) {
            // Leave central courtyard and main cross-axes open
            if (std::abs(gx) <= config.courtyardRadius && std::abs(gz) <= config.courtyardRadius) {
                continue;
            }
            // Leave 4 main avenues from spawn/center open for exploration
            if (gx == 0 || gz == 0) {
                continue;
            }

            uint32_t val = hashCoords(gx, gz, config.seed);
            glm::vec3 pos(gx * config.cellSize, 0.0f, gz * config.cellSize);

            if (val % 7 == 0) {
                // Tall Pillar
                float h = config.wallHeight * 0.9f;
                auto pillar = std::make_shared<Entity>();
                pillar->name = "Pillar_" + std::to_string(obstacleCount++);
                pillar->position = glm::vec3(pos.x, h * 0.5f, pos.z);
                pillar->scale = glm::vec3(1.4f, h, 1.4f);
                pillar->addComponent(std::make_shared<RendererComponent>(pillarModel));

                auto phys = std::make_shared<PhysicsComponent>();
                phys->setBox(pillar->scale * 0.5f);
                phys->setStatic(true);
                pillar->addComponent(phys);

                rootEntity->addChild(pillar);
            } else if (val % 5 == 0) {
                // Wall Section
                float h = config.wallHeight * 0.75f;
                auto wall = std::make_shared<Entity>();
                wall->name = "WallBlock_" + std::to_string(obstacleCount++);
                wall->position = glm::vec3(pos.x, h * 0.5f, pos.z);
                wall->scale = glm::vec3(config.cellSize, h, config.cellSize);
                wall->addComponent(std::make_shared<RendererComponent>(wallModel));

                auto phys = std::make_shared<PhysicsComponent>();
                phys->setBox(wall->scale * 0.5f);
                phys->setStatic(true);
                wall->addComponent(phys);

                rootEntity->addChild(wall);
            } else if (val % 11 == 0) {
                // Low barrier / bouncing obstacle
                float h = 1.2f;
                auto barrier = std::make_shared<Entity>();
                barrier->name = "Barrier_" + std::to_string(obstacleCount++);
                barrier->position = glm::vec3(pos.x, h * 0.5f, pos.z);
                barrier->scale = glm::vec3(config.cellSize * 0.8f, h, config.cellSize * 0.8f);
                barrier->addComponent(std::make_shared<RendererComponent>(wallModel));

                auto phys = std::make_shared<PhysicsComponent>();
                phys->setBox(barrier->scale * 0.5f);
                phys->setStatic(true);
                barrier->addComponent(phys);

                rootEntity->addChild(barrier);
            }
        }
    }

    // 8. Atmospheric Point Lights
    struct LightInfo {
        glm::vec3 pos;
        glm::vec3 col;
        float intensity;
    };

    std::vector<LightInfo> lights = {
        // Courtyard central warm beacon (offset slightly for dramatic diagonal shadows)
        { { 1.2f, 6.0f, 1.2f }, { 1.0f, 0.95f, 0.85f }, 0.80f },
        // Courtyard North / South accents
        { { 0.0f, 4.8f, -cDist * 0.85f }, { 0.3f, 0.85f, 1.0f }, 0.55f },
        { { 0.0f, 4.8f, cDist * 0.85f }, { 1.0f, 0.60f, 0.2f }, 0.55f },
        // Courtyard East / West accents
        { { cDist * 0.85f, 4.8f, 0.0f }, { 0.95f, 0.40f, 1.0f }, 0.55f },
        { { -cDist * 0.85f, 4.8f, 0.0f }, { 0.2f, 1.0f, 0.65f }, 0.55f },
        // Outer arena quadrant lights
        { { -halfSize * 0.6f, 5.0f, -halfSize * 0.6f }, { 1.0f, 0.95f, 0.55f }, 0.45f },
        { { halfSize * 0.6f, 5.0f, halfSize * 0.6f }, { 0.45f, 0.85f, 1.0f }, 0.45f }
    };

    for (size_t i = 0; i < lights.size(); ++i) {
        auto lightEnt = std::make_shared<Entity>();
        lightEnt->name = "PointLight_" + std::to_string(i);
        lightEnt->position = lights[i].pos;

        // Light component for renderer
        lightEnt->addComponent(std::make_shared<LightComponent>(lights[i].col, lights[i].intensity));

        // Visible glowing orb for the light
        lightEnt->addComponent(std::make_shared<RendererComponent>(orbModel));

        rootEntity->addChild(lightEnt);
    }

    std::cout << "ProceduralWorldGenerator: Placed " << obstacleCount << " obstacles and " << lights.size() << " point lights." << std::endl;
}
