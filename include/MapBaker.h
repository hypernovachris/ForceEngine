#ifndef MAP_BAKER_H
#define MAP_BAKER_H

#include <string>
#include <vector>
#include <glm/glm/glm.hpp>
#include "MapBinary.h"
#include "Mesh.h"

class MapBaker {
public:
    MapBaker();

    void setPlayerStart(const glm::vec3& pos, float yaw = 0.0f, float pitch = 0.0f);
    void setWorldBounds(float floorY, float minX, float maxX, float minZ, float maxZ);

    uint32_t registerMaterial(const std::string& materialPath);

    void addBox(const glm::vec3& center, const glm::vec3& size, uint32_t materialIndex,
                const glm::vec2& uvScale = glm::vec2(1.0f), bool hasCollision = true);

    void addCollider(const glm::vec3& center, const glm::vec3& halfExtents);
    void addTrigger(const glm::vec3& center, const glm::vec3& halfExtents, uint32_t eventId, const std::string& tag);
    void addLight(const glm::vec3& pos, const glm::vec3& col, float intensity, bool castsShadow = true);

    bool saveToFile(const std::string& filepath);

    // Baker for the 3D procedural arena into compiled .fmap binary
    static bool bakeProceduralCourtyard(const std::string& outputPath);

private:
    struct MeshBucket {
        uint32_t materialIndex;
        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
    };

    FMapHeader m_header;
    std::vector<std::string> m_materials;
    std::vector<MeshBucket> m_meshBuckets;
    std::vector<FMapCollider> m_colliders;
    std::vector<FMapTrigger> m_triggers;
    std::vector<FMapLight> m_lights;

    MeshBucket& getOrCreateBucket(uint32_t matIndex);
};

#endif // MAP_BAKER_H
