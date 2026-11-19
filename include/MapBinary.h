#ifndef MAP_BINARY_H
#define MAP_BINARY_H

#include <cstdint>
#include <glm/glm/glm.hpp>

#pragma pack(push, 1)

// Map Binary Header (Fixed 64 bytes)
struct FMapHeader {
    char magic[4];             // "FMAP"
    uint32_t version;          // 1
    
    // Player spawn transform
    glm::vec3 playerStart;
    float playerStartYaw;
    float playerStartPitch;

    // World floor & boundary limits
    float floorY;
    float minX;
    float maxX;
    float minZ;
    float maxZ;

    // Counts for contiguous array sections
    uint32_t numMaterials;
    uint32_t numMeshes;
    uint32_t numColliders;
    uint32_t numTriggers;
    uint32_t numLights;
};

// Fixed-size material reference
struct FMapMaterialRef {
    char path[128];
};

// Mesh section header
struct FMapMeshHeader {
    uint32_t materialIndex;    // Index into FMapMaterialRef array
    uint32_t vertexCount;      // Number of Vertex structs
    uint32_t indexCount;       // Number of uint32_t indices
};

// Static Box Collider (AABB)
struct FMapCollider {
    glm::vec3 center;
    glm::vec3 halfExtents;
};

// Gameplay / Event Trigger Volume
struct FMapTrigger {
    glm::vec3 center;
    glm::vec3 halfExtents;
    uint32_t eventId;
    char tag[32];
};

// Point Light
struct FMapLight {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;
    uint32_t castsShadow;      // 1 = true, 0 = false
};

#pragma pack(pop)

#endif // MAP_BINARY_H
