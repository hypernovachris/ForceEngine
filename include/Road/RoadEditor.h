#ifndef ROAD_EDITOR_H
#define ROAD_EDITOR_H

#include "Mesh.h"
#include "RHI/RHI_Pipeline.h"
#include "Road/RoadBaker.h"
#include "Road/RoadElevationSolver.h"
#include "Road/RoadRenderer.h"
#include "Road/RoadTypes.h"
#include "Road/VehiclePhysics.h"
#include "Shader.h"
#include "Terrain/QuadsphereLoader.h"
#include "Terrain/QuadsphereMath.h"
#include "Terrain/TerrainLoader.h"
#include "Terrain/TerrainTypes.h"
#include <SDL3/SDL.h>
#include <glm/glm/glm.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class Engine;

enum class EditorViewMode { TopDown2D, Orbit3D, DriveTest };
enum class EditorTool { Select, Pan };

class RoadEditor {
public:
  RoadEditor();
  ~RoadEditor();

  bool init(Engine &engine);
  void handleEvent(const SDL_Event &event);
  void update(float dt);
  void render(Engine &engine);

  void loadRawGeoJson(const std::string &path = "jughandle/export.geojson");
  void rebuildRoadGeometry();
  void
  export500mChunk(const std::string &path = "assets/maps/jughandle_500m.froad");
  void loadCompiledChunk(
      const std::string &path = "assets/maps/jughandle_500m.froad");
  void loadChunk(uint8_t face, int tx, int ty);
  void loadChunk(int cx, int cz);
  void saveActiveChunk();
  uint8_t getActiveChunkFace() const { return m_activeFace; }
  int getActiveChunkX() const { return m_activeChunkX; }
  int getActiveChunkZ() const { return m_activeChunkZ; }
  bool isQuadsphereChunk() const { return m_isQuadsphereChunk; }
  bool openChunkInputDialog(uint8_t &outFace, int &outX, int &outZ,
                            bool &outIsQuad);

  void enterDriveMode(int segmentIndex = -1);
  void exitDriveMode();

  bool m_enableInternalUI = false;

  // View & Tool Mode
  EditorViewMode getViewMode() const { return m_viewMode; }
  void setViewMode(EditorViewMode mode);
  EditorTool getActiveTool() const { return m_activeTool; }
  void setActiveTool(EditorTool tool) { m_activeTool = tool; }
  bool isKeyDown(SDL_Scancode scancode) const {
    if (static_cast<size_t>(scancode) < SDL_SCANCODE_COUNT) return m_keyState[scancode];
    return false;
  }
  void setKeyState(SDL_Scancode scancode, bool down) {
    if (static_cast<size_t>(scancode) < SDL_SCANCODE_COUNT) m_keyState[scancode] = down;
  }
  bool isAddingJunction() const { return m_isAddingJunction; }
  void setAddingJunction(bool adding);
  void cancelAddingJunction();
  void createJunctionFromPending();

  // Selection
  int getSelectedSegmentIdx() const { return m_selectedSegmentIdx; }
  void selectSegment(int idx);
  int getSelectedJunctionIdx() const { return m_selectedJunctionIdx; }
  void selectJunction(int idx);
  int getSelectedControlPointIdx() const { return m_selectedControlPointIdx; }
  void selectControlPoint(int idx) { m_selectedControlPointIdx = idx; }

  // Road & Junction data access
  const std::vector<ProcessedRoadSegment> &getSegments() const {
    return m_segments;
  }
  std::vector<ProcessedRoadSegment> &getSegments() { return m_segments; }
  const std::vector<RoadJunction> &getJunctions() const { return m_junctions; }
  std::vector<RoadJunction> &getJunctions() { return m_junctions; }
  const ProcessedRoadSegment *getSelectedSegment() const;
  ProcessedRoadSegment *getSelectedSegment();
  const RoadJunction *getSelectedJunction() const;
  RoadJunction *getSelectedJunction();

  // Road Modification Tools
  void removeJunction(int juncId);
  void deleteSegment(int segIdx);
  void disconnectSegmentFromJunction(int segIdx, bool isStart);
  void splitSegmentAtControlPoint(int segIdx, int cpIdx);
  void insertControlPointBefore(int segIdx, int cpIdx);
  void insertControlPointAfter(int segIdx, int cpIdx);
  void deleteControlPoint(int segIdx, int cpIdx);
  void setControlPointElevation(int segIdx, int cpIdx, float elevation);
  void rebuildSegmentGeometry(int segIdx);
  void rebuildTopologyAndGeometry();
  void applyAashtoStandards();

  // Elevation Solver
  RoadElevationSolverOptions &getSolverOptions() { return m_solverOptions; }
  const RoadElevationSolverOptions &getSolverOptions() const {
    return m_solverOptions;
  }
  bool isAutoSolveElevation() const { return m_autoSolveElevation; }
  void setAutoSolveElevation(bool enable) { m_autoSolveElevation = enable; }
  int getLastSolveSweeps() const { return m_lastSolveSweeps; }
  float getLastSolveTimeMs() const { return m_lastSolveTimeMs; }
  void triggerElevationSolve() { solveRoadElevations(); }

  // Chunk & Terrain Information
  int getTotalSegments() const { return static_cast<int>(m_segments.size()); }
  int getTotalJunctions() const { return static_cast<int>(m_junctions.size()); }
  size_t getAdjacentChunkCount() const { return m_adjacentMeshes.size(); }
  glm::dvec3 getChunkCenterECEF() const { return m_chunkCenterECEF; }
  bool hasTerrain() const { return m_hasTerrain; }
  bool isQuadsphere() const { return m_isQuadsphere; }
  int getMeshVertexCount() const;
  int getMeshTriangleCount() const;
  const std::string &getStatusMessage() const { return m_statusMessage; }
  void setStatusMessage(const std::string &msg, float time = 3.0f) {
    m_statusMessage = msg;
    m_statusTimer = time;
  }
  void recenterOrbitOnTerrain();
  void resetCameraFocus();
  void reloadActiveChunk();
  void createNewSegment();
  void testDriveSelectedRoad();

private:
  struct AdjacentChunkContext {
    std::shared_ptr<Mesh> mesh;
    glm::mat4 modelMatrix{1.0f};
  };

  Engine *m_engine = nullptr;

  EditorViewMode m_viewMode = EditorViewMode::Orbit3D;
  EditorTool m_activeTool = EditorTool::Select;

  // Road Data
  uint8_t m_activeFace = 2;
  uint8_t m_activeLOD = 15;
  bool m_isQuadsphereChunk = true;
  int m_activeChunkX = 16040;
  int m_activeChunkZ = 20958;
  glm::dvec3 m_chunkCenterECEF{0.0};
  glm::mat4 m_R_ecef_to_enu{1.0f};
  glm::mat4 m_R_enu_to_ecef{1.0f};
  std::string m_chunksDir = "assets/maps/somerset_chunks";
  std::vector<RawRoadFeature> m_rawFeatures;
  std::vector<ProcessedRoadSegment> m_segments;
  std::vector<RoadJunction> m_junctions;
  std::shared_ptr<Mesh> m_mergedMesh;
  std::shared_ptr<Mesh> m_selectedMesh;
  std::vector<AdjacentChunkContext> m_adjacentMeshes;

  int m_selectedSegmentIdx = -1;
  int m_selectedJunctionIdx = -1;

  // Tweak Parameters
  float m_laneWidth = 3.65f;     // meters
  float m_shoulderWidth = 1.50f; // meters
  float m_sampleStep = 2.50f;    // meters
  bool m_showGrid = true;
  bool m_show500mBox = true;
  bool m_filter500mOnly = false;
  bool m_showSplines = false;

  // 1m Grid & Road Chunk Terrain
  std::shared_ptr<Mesh> m_gridMesh;
  std::shared_ptr<Mesh> m_terrainMesh;
  std::shared_ptr<Mesh> m_flatGridMesh;
  std::shared_ptr<Shader> m_gridShader;
  std::shared_ptr<RHI_Pipeline> m_gridPipeline;

  // Terrain Container & Heightfield Data
  LoadedTerrainPack m_terrainPack;
  std::unordered_map<uint64_t, DecodedTerrainTile> m_tileCache;
  LoadedQuadspherePack m_quadPack;
  std::unordered_map<uint64_t, DecodedQuadsphereTile> m_quadTileCache;
  bool m_isQuadsphere = false;
  bool m_hasTerrain = false;
  bool m_showTerrain = true;

  // Physics Elevation Solver
  RoadElevationSolverOptions m_solverOptions;
  bool m_autoSolveElevation = false;
  int m_lastSolveSweeps = 0;
  float m_lastSolveTimeMs = 0.0f;
  int m_rightPanelTab =
      0; // 0 = Elevation Solver, 1 = Road Inspector, 2 = Junction Inspector
  float m_rightPanelHeight = 720.0f;
  float m_rightPanelScrollY = 0.0f;

  // Road Shader Pipeline & Drawing
  RoadRenderer m_roadRenderer;

  // Vehicle for passability test
  VehiclePhysics m_vehicle;

  // Camera State
  glm::vec3 m_cameraFocus{0.0f, 0.0f, 0.0f};
  float m_orbitDistance = 250.0f;
  float m_orbitPitch = 45.0f; // degrees
  float m_orbitYaw = 0.0f;    // degrees

  glm::vec3 m_topDownPos{0.0f, 350.0f, 0.0f};

  // Mouse / Interaction State
  float m_mouseX = 0.0f;
  float m_mouseY = 0.0f;
  bool m_mouseLeftDown = false;
  bool m_mouseLeftClicked = false;
  bool m_mouseRightDown = false;
  bool m_mouseMiddleDown = false;
  float m_mouseDeltaX = 0.0f;
  float m_mouseDeltaY = 0.0f;
  float m_mouseWheel = 0.0f;
  bool m_keyState[SDL_SCANCODE_COUNT] = {false};

  // Control Point Editing
  int m_selectedControlPointIdx = -1;
  int m_draggedControlPointIdx = -1;
  bool m_isDraggingControlPoint = false;
  glm::vec2 m_dragStartMouseScreen{0.0f, 0.0f};
  glm::vec3 m_dragStartControlPointPos{0.0f, 0.0f, 0.0f};
  glm::vec3 m_dragStartGroundHit{0.0f, 0.0f, 0.0f};
  bool m_dragThresholdPassed = false;

  // HiDPI Scaling
  float m_displayScale = 1.0f;
  float m_uiScale = 1.0f;

  // Interactive Junction Creation Tool
  struct PendingJunctionConnection {
    int segmentIdx = -1;
    bool isStart = true;
  };
  bool m_isAddingJunction = false;
  std::vector<PendingJunctionConnection> m_pendingJunctionConns;

  std::string m_statusMessage = "Loaded jughandle/export.geojson";
  float m_statusTimer = 5.0f;

  void initGrid();
  void initTerrain();
  void loadChunkImpl(const std::string &path, uint8_t face, int tx, int ty,
                     bool isQuad);
  void buildChunkTerrainMesh();
  float sampleTerrainHeight(float localX, float localZ);
  bool getTerrainHeight(float localX, float localZ, float &outHeight);
  void solveRoadElevations();
  void initRoadPipeline();
  void renderUI(int screenW, int screenH);
  void drawControlPoints(int screenW, int screenH);
  void handleJunctionSelectionClick(float mx, float my, int screenW,
                                    int screenH);
  glm::vec3 getGroundRayHit(float mx, float my, int screenW, int screenH,
                            float planeY = 0.05f) const;
  glm::mat4 getViewMatrix() const;
  glm::mat4 getProjMatrix(float aspect) const;
  void raycastSelectRoad(float mouseX, float mouseY, int screenW, int screenH);
};

#endif // ROAD_EDITOR_H
