#ifndef TERRAIN_LOADER_H
#define TERRAIN_LOADER_H

#include <string>
#include <vector>
#include <memory>
#include "Terrain/TerrainTypes.h"

class TerrainLoader {
public:
    // Open a hierarchical quadtree .fterrainpack container and load its Morton index table
    static bool openPack(const std::string& filepath, LoadedTerrainPack& outPack);

    // Fast O(log N) binary search lookup for a tile in the container
    static const FTerrainTileIndexEntry* findTileIndex(const LoadedTerrainPack& pack, uint16_t lod, uint32_t tileX, uint32_t tileY);

    // Stream and decompress a specific tile (LOD, X, Y) from the single container file
    static bool loadTile(const LoadedTerrainPack& pack, uint16_t lod, uint32_t tileX, uint32_t tileY, DecodedTerrainTile& outTile);

    // Sample height with bilinear interpolation inside a decoded 129x129 tile
    static float sampleTileHeight(const DecodedTerrainTile& tile, float localX, float localZ, float tileSize);

    // Optical/Physical Earth curvature drop: drop = d^2 / (2 * R)
    static float calculateEarthCurvatureDrop(float distanceMeters, float earthRadius = 6371000.0f);

    // Interleave 32-bit coordinates into a 64-bit Morton Z-order curve code
    static uint64_t morton2D(uint32_t x, uint32_t y);

    // Pack LOD, TileX, and TileY into a 64-bit key
    static uint64_t packTileKey(uint16_t lod, uint32_t tileX, uint32_t tileY);

    // Geodetic to ECEF, ECEF to local ENU, and inverse transforms
    static glm::dvec3 geodeticToECEF(double latDeg, double lonDeg, double heightMeters);
    static glm::vec3 ecefToENU(const glm::dvec3& ecefPoint, const glm::dvec3& ecefOrigin, const float* enuRotation3x3);
    static glm::dvec3 enuToECEF(const glm::vec3& enuPoint, const glm::dvec3& ecefOrigin, const float* enuRotation3x3);
    static glm::dvec3 ecefToGeodetic(const glm::dvec3& ecefPoint);

    // Frustum extraction and culling for perspectiveRH_ZO (Metal / Vulkan / DirectX depth [0, 1])
    static std::array<FrustumPlane, 6> extractFrustumPlanes(const glm::mat4& vp,
                                                           const glm::vec3& camPos,
                                                           const glm::vec3& camForward,
                                                           float nearZ = 0.5f,
                                                           float farZ = 20000.0f);
    static std::array<FrustumPlane, 6> extractFrustumPlanes(const glm::mat4& vp);
    static bool isBoxInFrustum(const std::array<FrustumPlane, 6>& planes, const glm::vec3& boxMin, const glm::vec3& boxMax);
};

#endif // TERRAIN_LOADER_H
