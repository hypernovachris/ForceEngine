#ifndef ROAD_TERRAIN_VIEWER_H
#define ROAD_TERRAIN_VIEWER_H

#include <vector>
#include <string>
#include <memory>
#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "Terrain/TerrainRenderer.h"
#include "Road/RoadRenderer.h"
#include "Road/RoadTypes.h"
#include "Road/RoadGenerator.h"
#include "Road/GeoJsonLoader.h"
#include "Mesh.h"

class Engine;

enum class ViewerCameraMode {
    FirstPersonGround, // Realistic human controller 1.6m above ground/road
    FlyDrone           // 6-DOF free camera for aerial highway network inspection
};

class RoadTerrainViewer {
public:
    RoadTerrainViewer();
    ~RoadTerrainViewer();

    bool init(Engine& engine,
              const std::string& terrainPackPath = "jughandle/somerset_mc_lerc.fterrainpack",
              const std::string& roadGeoJsonPath = "jughandle/export.geojson");

    bool loadTerrainPack(const std::string& packPath);
    bool loadAndDrapeRoads(const std::string& geojsonPath);

    void handleEvent(const SDL_Event& event);
    void update(float dt);
    void render(Engine& engine);

    // Camera Accessors
    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjMatrix(float aspect) const;
    glm::vec3 getCameraPosition() const;
    glm::vec3 getCameraForward() const;

    // Controls & Teleportation
    void setMouseLocked(bool locked);
    bool isMouseLocked() const { return m_mouseLocked; }
    void setCameraMode(ViewerCameraMode mode);
    ViewerCameraMode getCameraMode() const { return m_cameraMode; }
    bool teleportToCoordinates(double targetLat, double targetLon, float altitudeOffset = 1.6f);
    float sampleGroundElevation(float worldX, float worldZ);
    bool hasGroundElevation(float worldX, float worldZ, float& outElev);
    bool findRoadElevation(float worldX, float worldZ, float& outElev) const;
    bool openCoordinateDialog(double& outLat, double& outLon);
    void promptAndTeleport();

    const LoadedTerrainPack& getTerrainPack() const { return m_terrainRenderer.getPack(); }
    const std::vector<ProcessedRoadSegment>& getRoadSegments() const { return m_roadSegments; }
    const std::vector<RoadJunction>& getRoadJunctions() const { return m_roadJunctions; }
    std::shared_ptr<Mesh> getMergedRoadMesh() const { return m_mergedRoadMesh; }

private:
    void updateCamera(float dt);
    void renderUI(int screenW, int screenH);

    SDL_Window* m_window = nullptr;

    // Shared Quadtree Terrain Renderer
    TerrainRenderer m_terrainRenderer;

    // Shared Road Mesh Renderer
    RoadRenderer m_roadRenderer;

    // Road Data & Network Resources
    std::string m_chunksDir = "assets/maps/somerset_chunks";
    std::string m_roadGeoJsonPath;
    std::vector<RawRoadFeature> m_rawRoadFeatures;
    std::vector<ProcessedRoadSegment> m_roadSegments;
    std::vector<RoadJunction> m_roadJunctions;
    std::shared_ptr<Mesh> m_mergedRoadMesh;
    std::map<std::pair<int, int>, std::shared_ptr<Mesh>> m_loadedChunkMeshes;
    int m_lastPlayerChunkX = 999999;
    int m_lastPlayerChunkZ = 999999;

    struct LoadedQuadsphereRoadChunk {
        glm::dvec3 chunkCenterECEF{0.0};
        std::shared_ptr<Mesh> mesh;
        std::vector<ProcessedRoadSegment> segments;
    };
    std::map<std::tuple<uint8_t, uint32_t, uint32_t>, LoadedQuadsphereRoadChunk> m_loadedQuadMeshes;
    int m_lastPlayerQuadTx = -999999;
    int m_lastPlayerQuadTy = -999999;

    void updateChunkStreaming(float playerX, float playerZ);

    // Camera & Player Controller State
    ViewerCameraMode m_cameraMode = ViewerCameraMode::FirstPersonGround;
    glm::vec3 m_playerPos = glm::vec3(0.0f);
    glm::vec3 m_camPos = glm::vec3(0.0f, 1.6f, 0.0f);
    float m_yaw = -90.0f;
    float m_pitch = -8.0f;
    float m_eyeHeight = 1.6f;
    float m_walkSpeed = 1.4f;
    float m_sprintSpeed = 5.0f;
    float m_droneSpeed = 45.0f;
    float m_droneFastSpeed = 180.0f;
    float m_currentSpeed = 0.0f;

    // Real-Life Earth Coordinates (WGS84 Geodetic)
    double m_currentLat = 0.0;
    double m_currentLon = 0.0;
    double m_currentAlt = 0.0;
    float m_groundElevation = 0.0f;

    // Road Metrics
    float m_totalRoadLengthMeters = 0.0f;
    uint32_t m_totalRoadVertices = 0;
    uint32_t m_totalRoadTriangles = 0;
    uint32_t m_elevatedBridgeCount = 0;

    // Input States
    bool m_mouseLocked = true;
    bool m_keyW = false;
    bool m_keyS = false;
    bool m_keyA = false;
    bool m_keyD = false;
    bool m_keyQ = false;
    bool m_keyE = false;
    bool m_keySpace = false;
    bool m_keyShift = false;
    bool m_keyCtrl = false;
    bool m_mouseRightDown = false;
    bool m_mouseLeftDown = false;
    float m_mouseX = 0.0f;
    float m_mouseY = 0.0f;
    float m_mouseWheel = 0.0f;
    bool m_mouseLeftClicked = false;

    // Layer Visibility
    bool m_renderRoads = true;
    bool m_renderTerrain = true;
    float m_uiScale = 1.0f;
};

#endif // ROAD_TERRAIN_VIEWER_H
