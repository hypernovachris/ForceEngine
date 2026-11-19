#include "MapLoader.h"
#include "Engine.h"
#include "ResourceManager.h"
#include "RendererComponent.h"
#include "PhysicsComponent.h"
#include "PhysicsSystem.h"
#include "LightComponent.h"
#include "TriggerSystem.h"
#include "PrimitiveBuilder.h"
#include <cstdio>
#include <iostream>

MapLoadResult MapLoader::loadMap(const std::string& filepath, Engine& engine) {
    MapLoadResult result = {};

    FILE* file = std::fopen(filepath.c_str(), "rb");
    if (!file) {
        std::cerr << "MapLoader: Failed to open map file: " << filepath << std::endl;
        return result;
    }

    // 1. Read and validate Header
    FMapHeader header = {};
    if (std::fread(&header, sizeof(FMapHeader), 1, file) != 1) {
        std::cerr << "MapLoader: Failed to read header from: " << filepath << std::endl;
        std::fclose(file);
        return result;
    }

    if (header.magic[0] != 'F' || header.magic[1] != 'M' ||
        header.magic[2] != 'A' || header.magic[3] != 'P') {
        std::cerr << "MapLoader: Invalid magic header in: " << filepath << std::endl;
        std::fclose(file);
        return result;
    }

    std::cout << "MapLoader: Ingesting binary map " << filepath << " (v" << header.version << ")..." << std::endl;

    // Set world floor & bounds in PhysicsSystem
    PhysicsSystem::getInstance().setFloorLevel(header.floorY);
    PhysicsSystem::getInstance().setWorldBounds(header.minX, header.maxX, header.minZ, header.maxZ);

    // 2. Read Material References
    std::vector<std::shared_ptr<Material>> materials(header.numMaterials);
    for (uint32_t i = 0; i < header.numMaterials; ++i) {
        FMapMaterialRef ref = {};
        std::fread(&ref, sizeof(FMapMaterialRef), 1, file);
        materials[i] = ResourceManager::loadForceMaterial(ref.path);
        if (!materials[i]) {
            std::cerr << "MapLoader: Warning - failed to load material: " << ref.path << std::endl;
        }
    }

    // 3. Read Pre-merged Static Meshes into a Single World Model
    auto worldModel = std::make_shared<Model>();

    for (uint32_t i = 0; i < header.numMeshes; ++i) {
        FMapMeshHeader meshHeader = {};
        std::fread(&meshHeader, sizeof(FMapMeshHeader), 1, file);

        std::vector<Vertex> vertices(meshHeader.vertexCount);
        std::vector<uint32_t> indices(meshHeader.indexCount);

        if (meshHeader.vertexCount > 0) {
            std::fread(vertices.data(), sizeof(Vertex), meshHeader.vertexCount, file);
        }
        if (meshHeader.indexCount > 0) {
            std::fread(indices.data(), sizeof(uint32_t), meshHeader.indexCount, file);
        }

        // Zero-copy direct upload to GPU buffer
        auto mesh = std::make_shared<Mesh>(vertices.data(), meshHeader.vertexCount,
                                           indices.data(), meshHeader.indexCount);
        worldModel->meshes.push_back(mesh);

        auto mat = (meshHeader.materialIndex < materials.size()) ? materials[meshHeader.materialIndex] : nullptr;
        worldModel->materials.push_back(mat);
    }

    // Attach single combined world geometry entity
    if (!worldModel->meshes.empty() && engine.rootEntity) {
        auto worldEntity = std::make_shared<Entity>();
        worldEntity->name = "World_Geometry";
        worldEntity->position = glm::vec3(0.0f);
        worldEntity->scale = glm::vec3(1.0f);
        worldEntity->addComponent(std::make_shared<RendererComponent>(worldModel));
        engine.rootEntity->addChild(worldEntity);
    }

    // 4. Read Colliders and Register into PhysicsSystem
    std::vector<FMapCollider> colliders(header.numColliders);
    if (header.numColliders > 0) {
        std::fread(colliders.data(), sizeof(FMapCollider), header.numColliders, file);

        for (uint32_t i = 0; i < header.numColliders; ++i) {
            auto colEnt = std::make_shared<Entity>();
            colEnt->name = "StaticCol_" + std::to_string(i);
            colEnt->position = colliders[i].center;

            auto phys = std::make_shared<PhysicsComponent>();
            phys->setBox(colliders[i].halfExtents);
            phys->setStatic(true);
            colEnt->addComponent(phys);

            engine.rootEntity->addChild(colEnt);
        }
    }

    // 5. Read Triggers and Register into TriggerSystem
    std::vector<FMapTrigger> triggers(header.numTriggers);
    TriggerSystem::getInstance().clear();
    if (header.numTriggers > 0) {
        std::fread(triggers.data(), sizeof(FMapTrigger), header.numTriggers, file);
        for (const auto& trig : triggers) {
            TriggerSystem::getInstance().registerTrigger(trig);
        }
    }

    // 6. Read Lights and Spawn Light Entities with Visual Orbs
    std::vector<FMapLight> lights(header.numLights);
    if (header.numLights > 0) {
        std::fread(lights.data(), sizeof(FMapLight), header.numLights, file);

        auto sunMat = ResourceManager::loadForceMaterial("assets/materials/sun.ForceMaterial");
        if (sunMat) {
            sunMat->castsShadow = false;
        }
        auto orbSphere = PrimitiveBuilder::createSphere(12, 12, 0.25f, true, true);
        auto orbModel = std::make_shared<Model>();
        orbModel->meshes.push_back(orbSphere->meshes[0]);
        orbModel->materials.push_back(sunMat);

        for (uint32_t i = 0; i < header.numLights; ++i) {
            auto lightEnt = std::make_shared<Entity>();
            lightEnt->name = "PointLight_" + std::to_string(i);
            lightEnt->position = lights[i].position;

            lightEnt->addComponent(std::make_shared<LightComponent>(
                lights[i].color, lights[i].intensity, lights[i].castsShadow != 0));
            lightEnt->addComponent(std::make_shared<RendererComponent>(orbModel));

            engine.rootEntity->addChild(lightEnt);
        }
    }

    std::fclose(file);

    result.success = true;
    result.playerStart = header.playerStart;
    result.playerStartYaw = header.playerStartYaw;
    result.playerStartPitch = header.playerStartPitch;
    result.meshCount = header.numMeshes;
    result.colliderCount = header.numColliders;
    result.triggerCount = header.numTriggers;
    result.lightCount = header.numLights;

    std::cout << "MapLoader: Loaded " << filepath << " successfully! ("
              << result.meshCount << " merged draw chunks, "
              << result.colliderCount << " static colliders, "
              << result.triggerCount << " triggers, "
              << result.lightCount << " lights)" << std::endl;

    return result;
}
