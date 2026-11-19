#ifndef TERRAIN_RENDERER_H
#define TERRAIN_RENDERER_H

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <glm/glm/glm.hpp>
#include <SDL3/SDL_gpu.h>
#include "Terrain/TerrainTypes.h"
#include "Terrain/TerrainLoader.h"
#include "Terrain/QuadsphereLoader.h"
#include "Terrain/QuadsphereTypes.h"
#include "Terrain/QuadsphereMath.h"
#include "Shader.h"
#include "RHI/RHI_Pipeline.h"
#include "RHI/RHI_Buffer.h"

struct CachedTerrainMesh {
    std::unique_ptr<RHI_Buffer> vertexBuffer;
    uint64_t tileKey = 0;
    uint16_t lod = 0;
    uint32_t tileX = 0;
    uint32_t tileY = 0;
    uint64_t lastUsedFrame = 0;
};

struct CachedQuadsphereTile {
    SDL_GPUTexture* texture = nullptr;
    uint64_t tileKey = 0;
    uint8_t face = 0;
    uint16_t lod = 0;
    uint32_t tx = 0;
    uint32_t ty = 0;
    float tileMinZ = 0.0f;
    uint64_t lastUsedFrame = 0;
};

struct VisibleTerrainTile {
    uint16_t lod = 0;
    uint32_t tileX = 0;
    uint32_t tileY = 0;
    uint64_t tileKey = 0;
    float distanceToCam = 0.0f;
    glm::vec3 boundsMin = glm::vec3(0.0f);
    glm::vec3 boundsMax = glm::vec3(0.0f);
};

struct TerrainLoadRequest {
    uint16_t lod;
    uint32_t tx;
    uint32_t ty;
    uint64_t key;
};

struct TerrainUniforms {
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;
    glm::vec4 viewPos;
    glm::vec4 terrainParams; // [lodTint ? 1 : 0, baseTileSize, earthCurvature ? 1 : 0, 0]
};

class TerrainRenderer {
public:
    TerrainRenderer();
    ~TerrainRenderer();

    // Initialize graphics pipeline and shared index buffer with skirts
    bool init(SDL_GPUTextureFormat colorFormat,
              SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT);

    // Open and load a .fterrainpack container
    bool loadPack(const std::string& packPath);

    // Update quadtree LOD selection, streaming, and LRU cache eviction
    void update(const glm::vec3& camPos,
                const glm::vec3& camForward,
                const glm::mat4& view,
                const glm::mat4& proj,
                float aspect,
                float dt,
                const glm::dvec3* optCamECEF = nullptr);

    // Render all visible quadtree terrain tiles
    void render(SDL_GPURenderPass* pass,
                SDL_GPUCommandBuffer* cmdBuf,
                const glm::mat4& view,
                const glm::mat4& proj,
                const glm::vec3& camPos,
                const glm::dvec3* optCamECEF = nullptr);

    // Terrain queries
    float sampleElevation(float posX, float posZ) const;
    bool hasElevation(float posX, float posZ, float& outElevation) const;
    float sampleElevationGeodetic(double latDeg, double lonDeg) const;
    bool hasElevationGeodetic(double latDeg, double lonDeg, float& outElevation) const;
    void prewarmDestinationTiles(float posX, float posZ);

    // Configuration & Toggles
    void setLodColorTint(bool enable) { m_lodColorTint = enable; }
    bool getLodColorTint() const { return m_lodColorTint; }

    void setEarthCurvature(bool enable) { m_earthCurvature = enable; }
    bool getEarthCurvature() const { return m_earthCurvature; }

    void setFrustumCulling(bool enable) { m_enableFrustumCulling = enable; }
    bool getFrustumCulling() const { return m_enableFrustumCulling; }

    void setLodSplitMultiplier(float mult) { m_lodSplitMultiplier = mult; }
    float getLodSplitMultiplier() const { return m_lodSplitMultiplier; }

    // Telemetry & State Inspection
    bool isPackLoaded() const { return m_packLoaded; }
    bool isQuadsphere() const { return m_isQuadsphere; }
    const LoadedTerrainPack& getPack() const { return m_pack; }
    const LoadedQuadspherePack& getQuadspherePack() const { return m_quadPack; }
    const std::string& getPackPath() const { return m_packPath; }
    size_t getRenderedTileCount() const { return m_isQuadsphere ? m_quadRenderList.size() : m_renderList.size(); }
    size_t getCachedTileCount() const { return m_isQuadsphere ? m_quadGpuCache.size() : m_gpuTileCache.size(); }
    size_t getMaxCacheTiles() const { return m_maxCacheTiles; }
    uint32_t getSharedIndexCount() const { return m_isQuadsphere ? m_quadStaticIndexCount : m_sharedIndexCount; }
    const std::vector<uint32_t>& getLodCounts() const { return m_lodCounts; }
    const std::vector<VisibleTerrainTile>& getVisibleTiles() const { return m_visibleTiles; }
    const std::vector<const CachedTerrainMesh*>& getRenderList() const { return m_renderList; }

private:
    void buildSharedIndexBuffer();
    void buildStaticQuadsphereMesh();
    void stitchTileMegachunkSeams(uint16_t lod, uint32_t tx, uint32_t ty, DecodedTerrainTile& tile) const;
    void selectTileRecursive(uint16_t lod, uint32_t tx, uint32_t ty,
                             const glm::vec3& camPos,
                             const std::array<FrustumPlane, 6>& planes);
    void selectQuadsphereTileRecursive(uint8_t face, uint16_t lod, uint32_t tx, uint32_t ty,
                                       const glm::dvec3& camPosECEF,
                                       const std::array<FrustumPlane, 6>& planes);
    void requestTileLoad(uint16_t lod, uint32_t tx, uint32_t ty, uint64_t key);
    bool uploadTileMesh(uint16_t lod, uint32_t tx, uint32_t ty, uint64_t key);
    bool uploadQuadsphereTile(uint8_t face, uint16_t lod, uint32_t tx, uint32_t ty, uint64_t key);
    void evictOldMeshes();

    // RHI Resources
    std::shared_ptr<Shader> m_shader;
    std::shared_ptr<RHI_Pipeline> m_pipeline;
    RHI_Buffer m_sharedIndexBuffer;
    uint32_t m_sharedIndexCount = 0;

    // Quadsphere RHI Resources
    std::shared_ptr<Shader> m_quadShader;
    std::shared_ptr<RHI_Pipeline> m_quadPipeline;
    RHI_Buffer m_quadStaticVertexBuffer;
    RHI_Buffer m_quadStaticIndexBuffer;
    uint32_t m_quadStaticIndexCount = 0;
    SDL_GPUSampler* m_quadSampler = nullptr;

    // Pack State
    LoadedTerrainPack m_pack;
    LoadedQuadspherePack m_quadPack;
    std::string m_packPath;
    bool m_packLoaded = false;
    bool m_isQuadsphere = false;

    // GPU Tile Cache & Streaming
    std::unordered_map<uint64_t, CachedTerrainMesh> m_gpuTileCache;
    std::unordered_map<uint64_t, CachedQuadsphereTile> m_quadGpuCache;
    std::vector<VisibleTerrainTile> m_visibleTiles;
    std::vector<const CachedTerrainMesh*> m_renderList;
    std::vector<const CachedQuadsphereTile*> m_quadRenderList;
    std::vector<TerrainLoadRequest> m_loadQueue;
    std::vector<uint32_t> m_lodCounts;

    size_t m_maxCacheTiles = 1024;
    uint64_t m_frameCounter = 0;

    // Parameters
    float m_lodSplitMultiplier = 4.5f;
    bool m_enableFrustumCulling = true;
    bool m_lodColorTint = false;
    bool m_earthCurvature = false;
};

#endif // TERRAIN_RENDERER_H
