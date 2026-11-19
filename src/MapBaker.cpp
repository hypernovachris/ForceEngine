#include "MapBaker.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <iostream>

MapBaker::MapBaker() {
    std::memset(&m_header, 0, sizeof(FMapHeader));
    m_header.magic[0] = 'F';
    m_header.magic[1] = 'M';
    m_header.magic[2] = 'A';
    m_header.magic[3] = 'P';
    m_header.version = 1;

    m_header.playerStart = glm::vec3(0.0f, 1.7f, 6.0f);
    m_header.playerStartYaw = 0.0f;
    m_header.playerStartPitch = 0.0f;

    m_header.floorY = 0.0f;
    m_header.minX = -100.0f;
    m_header.maxX = 100.0f;
    m_header.minZ = -100.0f;
    m_header.maxZ = 100.0f;
}

void MapBaker::setPlayerStart(const glm::vec3& pos, float yaw, float pitch) {
    m_header.playerStart = pos;
    m_header.playerStartYaw = yaw;
    m_header.playerStartPitch = pitch;
}

void MapBaker::setWorldBounds(float floorY, float minX, float maxX, float minZ, float maxZ) {
    m_header.floorY = floorY;
    m_header.minX = minX;
    m_header.maxX = maxX;
    m_header.minZ = minZ;
    m_header.maxZ = maxZ;
}

uint32_t MapBaker::registerMaterial(const std::string& materialPath) {
    for (uint32_t i = 0; i < (uint32_t)m_materials.size(); ++i) {
        if (m_materials[i] == materialPath) {
            return i;
        }
    }
    m_materials.push_back(materialPath);
    return (uint32_t)(m_materials.size() - 1);
}

MapBaker::MeshBucket& MapBaker::getOrCreateBucket(uint32_t matIndex) {
    for (auto& bucket : m_meshBuckets) {
        if (bucket.materialIndex == matIndex) {
            return bucket;
        }
    }
    m_meshBuckets.push_back(MeshBucket{matIndex, {}, {}});
    return m_meshBuckets.back();
}

void MapBaker::addBox(const glm::vec3& center, const glm::vec3& size, uint32_t materialIndex,
                     const glm::vec2& uvScale, bool hasCollision) {
    MeshBucket& bucket = getOrCreateBucket(materialIndex);
    uint32_t baseVertex = (uint32_t)bucket.vertices.size();

    float w = size.x * 0.5f;
    float h = size.y * 0.5f;
    float d = size.z * 0.5f;

    // 6 faces * 4 vertices = 24 vertices
    struct FaceData {
        glm::vec3 normal;
        glm::vec3 tangent;
        glm::vec3 p[4];
    };

    FaceData faces[6] = {
        // Front (+Z)
        { { 0.0f,  0.0f,  1.0f }, {  1.0f, 0.0f,  0.0f },
          { {-w, -h,  d}, { w, -h,  d}, { w,  h,  d}, {-w,  h,  d} } },
        // Back (-Z)
        { { 0.0f,  0.0f, -1.0f }, { -1.0f, 0.0f,  0.0f },
          { { w, -h, -d}, {-w, -h, -d}, {-w,  h, -d}, { w,  h, -d} } },
        // Left (-X)
        { {-1.0f,  0.0f,  0.0f }, {  0.0f, 0.0f,  1.0f },
          { {-w, -h, -d}, {-w, -h,  d}, {-w,  h,  d}, {-w,  h, -d} } },
        // Right (+X)
        { { 1.0f,  0.0f,  0.0f }, {  0.0f, 0.0f, -1.0f },
          { { w, -h,  d}, { w, -h, -d}, { w,  h, -d}, { w,  h,  d} } },
        // Top (+Y)
        { { 0.0f,  1.0f,  0.0f }, {  1.0f, 0.0f,  0.0f },
          { {-w,  h,  d}, { w,  h,  d}, { w,  h, -d}, {-w,  h, -d} } },
        // Bottom (-Y)
        { { 0.0f, -1.0f,  0.0f }, {  1.0f, 0.0f,  0.0f },
          { {-w, -h, -d}, { w, -h, -d}, { w, -h,  d}, {-w, -h,  d} } }
    };

    glm::vec2 texCoords[4] = {
        { 0.0f, 0.0f },
        { uvScale.x, 0.0f },
        { uvScale.x, uvScale.y },
        { 0.0f, uvScale.y }
    };

    for (int f = 0; f < 6; ++f) {
        uint32_t faceStart = (uint32_t)bucket.vertices.size();
        for (int v = 0; v < 4; ++v) {
            Vertex vert = {};
            glm::vec3 worldP = center + faces[f].p[v];
            vert.position[0] = worldP.x;
            vert.position[1] = worldP.y;
            vert.position[2] = worldP.z;

            vert.normal[0] = faces[f].normal.x;
            vert.normal[1] = faces[f].normal.y;
            vert.normal[2] = faces[f].normal.z;

            vert.texCoords[0] = texCoords[v].x;
            vert.texCoords[1] = texCoords[v].y;

            vert.tangent[0] = faces[f].tangent.x;
            vert.tangent[1] = faces[f].tangent.y;
            vert.tangent[2] = faces[f].tangent.z;

            bucket.vertices.push_back(vert);
        }

        // Two triangles per face
        bucket.indices.push_back(faceStart + 0);
        bucket.indices.push_back(faceStart + 1);
        bucket.indices.push_back(faceStart + 2);

        bucket.indices.push_back(faceStart + 2);
        bucket.indices.push_back(faceStart + 3);
        bucket.indices.push_back(faceStart + 0);
    }

    if (hasCollision) {
        addCollider(center, glm::vec3(w, h, d));
    }
}

void MapBaker::addCollider(const glm::vec3& center, const glm::vec3& halfExtents) {
    FMapCollider col = { center, halfExtents };
    m_colliders.push_back(col);
}

void MapBaker::addTrigger(const glm::vec3& center, const glm::vec3& halfExtents, uint32_t eventId, const std::string& tag) {
    FMapTrigger trig = {};
    trig.center = center;
    trig.halfExtents = halfExtents;
    trig.eventId = eventId;
    std::strncpy(trig.tag, tag.c_str(), sizeof(trig.tag) - 1);
    m_triggers.push_back(trig);
}

void MapBaker::addLight(const glm::vec3& pos, const glm::vec3& col, float intensity, bool castsShadow) {
    FMapLight light = { pos, col, intensity, castsShadow ? 1u : 0u };
    m_lights.push_back(light);
}

bool MapBaker::saveToFile(const std::string& filepath) {
    FILE* file = std::fopen(filepath.c_str(), "wb");
    if (!file) {
        std::cerr << "MapBaker: Failed to open file for writing: " << filepath << std::endl;
        return false;
    }

    // 1. Prepare Header
    m_header.numMaterials = (uint32_t)m_materials.size();
    m_header.numMeshes = (uint32_t)m_meshBuckets.size();
    m_header.numColliders = (uint32_t)m_colliders.size();
    m_header.numTriggers = (uint32_t)m_triggers.size();
    m_header.numLights = (uint32_t)m_lights.size();

    std::fwrite(&m_header, sizeof(FMapHeader), 1, file);

    // 2. Write Material References
    for (const auto& matPath : m_materials) {
        FMapMaterialRef ref = {};
        std::strncpy(ref.path, matPath.c_str(), sizeof(ref.path) - 1);
        std::fwrite(&ref, sizeof(FMapMaterialRef), 1, file);
    }

    // 3. Write Pre-merged Meshes
    for (const auto& bucket : m_meshBuckets) {
        FMapMeshHeader meshHeader = {};
        meshHeader.materialIndex = bucket.materialIndex;
        meshHeader.vertexCount = (uint32_t)bucket.vertices.size();
        meshHeader.indexCount = (uint32_t)bucket.indices.size();

        std::fwrite(&meshHeader, sizeof(FMapMeshHeader), 1, file);
        if (!bucket.vertices.empty()) {
            std::fwrite(bucket.vertices.data(), sizeof(Vertex), bucket.vertices.size(), file);
        }
        if (!bucket.indices.empty()) {
            std::fwrite(bucket.indices.data(), sizeof(uint32_t), bucket.indices.size(), file);
        }
    }

    // 4. Write Colliders
    if (!m_colliders.empty()) {
        std::fwrite(m_colliders.data(), sizeof(FMapCollider), m_colliders.size(), file);
    }

    // 5. Write Triggers
    if (!m_triggers.empty()) {
        std::fwrite(m_triggers.data(), sizeof(FMapTrigger), m_triggers.size(), file);
    }

    // 6. Write Lights
    if (!m_lights.empty()) {
        std::fwrite(m_lights.data(), sizeof(FMapLight), m_lights.size(), file);
    }

    std::fclose(file);
    std::cout << "MapBaker: Successfully baked " << filepath << " ("
              << m_meshBuckets.size() << " combined meshes, "
              << m_colliders.size() << " colliders, "
              << m_triggers.size() << " triggers, "
              << m_lights.size() << " lights)" << std::endl;
    return true;
}

// Deterministic hash helper for procedural arena placement
static uint32_t hashGrid(int x, int z, unsigned int seed) {
    uint32_t h = seed ^ (x * 73856093) ^ (z * 19349663);
    h = (h ^ (h >> 13)) * 0x5bd1e995;
    h ^= (h >> 15);
    return h;
}

bool MapBaker::bakeProceduralCourtyard(const std::string& outputPath) {
    MapBaker baker;

    int gridSize = 25;
    float cellSize = 2.0f;
    float wallH = 3.5f;
    int courtyardRadius = 4;
    unsigned int seed = 2026;

    float totalSize = gridSize * cellSize;
    float halfSize = totalSize * 0.5f;

    baker.setPlayerStart(glm::vec3(0.0f, 1.7f, 6.0f), 0.0f, 0.0f);
    baker.setWorldBounds(0.0f, -halfSize + 0.5f, halfSize - 0.5f, -halfSize + 0.5f, halfSize - 0.5f);

    // Register Materials
    uint32_t floorMat = baker.registerMaterial("assets/materials/glossy_tile.ForceMaterial");
    uint32_t wallMat  = baker.registerMaterial("assets/materials/tung.ForceMaterial");
    uint32_t pillarMat = baker.registerMaterial("assets/materials/david.ForceMaterial");

    // 1. Ground Floor (Floor does not need box collider because PhysicsSystem handles floorY)
    baker.addBox(glm::vec3(0.0f, -0.2f, 0.0f),
                 glm::vec3(totalSize + 2.0f, 0.4f, totalSize + 2.0f),
                 floorMat, glm::vec2(gridSize * 0.5f, gridSize * 0.5f), false);

    // 2. Perimeter Walls
    float wallThickness = 1.2f;
    // North
    baker.addBox(glm::vec3(0.0f, wallH * 0.5f, -halfSize),
                 glm::vec3(totalSize + wallThickness * 2.0f, wallH, wallThickness),
                 wallMat, glm::vec2(totalSize * 0.5f, 1.0f), true);
    // South
    baker.addBox(glm::vec3(0.0f, wallH * 0.5f, halfSize),
                 glm::vec3(totalSize + wallThickness * 2.0f, wallH, wallThickness),
                 wallMat, glm::vec2(totalSize * 0.5f, 1.0f), true);
    // West
    baker.addBox(glm::vec3(-halfSize, wallH * 0.5f, 0.0f),
                 glm::vec3(wallThickness, wallH, totalSize),
                 wallMat, glm::vec2(totalSize * 0.5f, 1.0f), true);
    // East
    baker.addBox(glm::vec3(halfSize, wallH * 0.5f, 0.0f),
                 glm::vec3(wallThickness, wallH, totalSize),
                 wallMat, glm::vec2(totalSize * 0.5f, 1.0f), true);

    // 3. Central Courtyard Raised Platform
    baker.addBox(glm::vec3(0.0f, 0.25f, 0.0f), glm::vec3(4.0f, 0.5f, 4.0f), pillarMat, glm::vec2(1.0f), true);

    // 4. Courtyard Framing Columns
    float cDist = courtyardRadius * cellSize;
    glm::vec2 corners[4] = { {-cDist, -cDist}, {cDist, -cDist}, {-cDist, cDist}, {cDist, cDist} };
    for (int i = 0; i < 4; ++i) {
        baker.addBox(glm::vec3(corners[i].x, wallH * 0.5f, corners[i].y),
                     glm::vec3(1.2f, wallH, 1.2f),
                     pillarMat, glm::vec2(1.0f), true);
    }

    // 5. Grid Obstacles (Outside Courtyard)
    int radius = gridSize / 2;
    for (int gx = -radius + 1; gx < radius; ++gx) {
        for (int gz = -radius + 1; gz < radius; ++gz) {
            if (std::abs(gx) <= courtyardRadius && std::abs(gz) <= courtyardRadius) continue;
            if (gx == 0 || gz == 0) continue;

            uint32_t val = hashGrid(gx, gz, seed);
            glm::vec3 pos(gx * cellSize, 0.0f, gz * cellSize);

            if (val % 7 == 0) {
                float h = wallH * 0.9f;
                baker.addBox(glm::vec3(pos.x, h * 0.5f, pos.z), glm::vec3(1.4f, h, 1.4f), pillarMat, glm::vec2(1.0f), true);
            } else if (val % 5 == 0) {
                float h = wallH * 0.75f;
                baker.addBox(glm::vec3(pos.x, h * 0.5f, pos.z), glm::vec3(cellSize, h, cellSize), wallMat, glm::vec2(1.0f), true);
            } else if (val % 11 == 0) {
                float h = 1.2f;
                baker.addBox(glm::vec3(pos.x, h * 0.5f, pos.z), glm::vec3(cellSize * 0.8f, h, cellSize * 0.8f), wallMat, glm::vec2(1.0f), true);
            }
        }
    }

    // 6. Gameplay Trigger Volumes
    // Courtyard Center Plaza Trigger
    baker.addTrigger(glm::vec3(0.0f, 1.5f, 0.0f), glm::vec3(3.0f, 2.0f, 3.0f), 1, "Courtyard_Plaza");
    // North Gate Trigger
    baker.addTrigger(glm::vec3(0.0f, 2.0f, -halfSize + 2.5f), glm::vec3(5.0f, 2.5f, 2.0f), 2, "North_Gate");
    // South Gate Trigger
    baker.addTrigger(glm::vec3(0.0f, 2.0f, halfSize - 2.5f), glm::vec3(5.0f, 2.5f, 2.0f), 3, "South_Gate");

    // 7. Atmospheric Point Lights
    baker.addLight(glm::vec3(1.2f, 6.0f, 1.2f), glm::vec3(1.0f, 0.95f, 0.85f), 0.80f, true);
    baker.addLight(glm::vec3(0.0f, 4.8f, -cDist * 0.85f), glm::vec3(0.3f, 0.85f, 1.0f), 0.55f, true);
    baker.addLight(glm::vec3(0.0f, 4.8f, cDist * 0.85f), glm::vec3(1.0f, 0.60f, 0.2f), 0.55f, true);
    baker.addLight(glm::vec3(cDist * 0.85f, 4.8f, 0.0f), glm::vec3(0.95f, 0.40f, 1.0f), 0.55f, true);
    baker.addLight(glm::vec3(-cDist * 0.85f, 4.8f, 0.0f), glm::vec3(0.2f, 1.0f, 0.65f), 0.55f, true);
    baker.addLight(glm::vec3(-halfSize * 0.6f, 5.0f, -halfSize * 0.6f), glm::vec3(1.0f, 0.95f, 0.55f), 0.45f, true);
    baker.addLight(glm::vec3(halfSize * 0.6f, 5.0f, halfSize * 0.6f), glm::vec3(0.45f, 0.85f, 1.0f), 0.45f, true);

    return baker.saveToFile(outputPath);
}
