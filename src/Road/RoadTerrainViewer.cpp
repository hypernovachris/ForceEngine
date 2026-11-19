#include "Road/RoadTerrainViewer.h"
#include "Engine.h"
#include "Road/RoadBinary.h"
#include "Road/RoadElevationSolver.h"
#include "UI/UIRenderer.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

RoadTerrainViewer::RoadTerrainViewer() = default;
RoadTerrainViewer::~RoadTerrainViewer() = default;

bool RoadTerrainViewer::init(Engine &engine, const std::string &terrainPackPath,
                             const std::string &roadGeoJsonPath) {
  m_window = engine.getWindow();
  m_roadGeoJsonPath = roadGeoJsonPath;

  // 1. Initialize Immediate-Mode UI
  if (!UI::init()) {
    std::cerr << "RoadTerrainViewer: Failed to initialize UI subsystem"
              << std::endl;
  }

  // 2. Initialize Shared Renderers (Terrain + Road Pipelines)
  auto &rhi = RHI_Device::getInstance();
  if (!m_terrainRenderer.init(rhi.getSwapchainFormat(),
                              SDL_GPU_TEXTUREFORMAT_D32_FLOAT)) {
    std::cerr << "RoadTerrainViewer: Failed to initialize TerrainRenderer"
              << std::endl;
    return false;
  }
  if (!m_roadRenderer.init(rhi.getSwapchainFormat(),
                           SDL_GPU_TEXTUREFORMAT_D32_FLOAT)) {
    std::cerr << "RoadTerrainViewer: Failed to initialize RoadRenderer"
              << std::endl;
    return false;
  }

  // 3. Load initial Terrain Pack (LERC)
  if (!loadTerrainPack(terrainPackPath)) {
    // Fallback to local jughandle LERC pack if somerset not present
    if (terrainPackPath != "jughandle/jughandle_lerc.fterrainpack" &&
        !loadTerrainPack("jughandle/jughandle_lerc.fterrainpack")) {
      std::cerr << "RoadTerrainViewer: Failed to load terrain pack "
                << terrainPackPath << std::endl;
      return false;
    }
  }

  // 4. Load Road Network (stream 3x3 chunks from somerset_chunks or
  // new_jersey_chunks if present)
  if (roadGeoJsonPath.find("somerset") != std::string::npos &&
      std::filesystem::exists("assets/maps/somerset_chunks") &&
      !std::filesystem::is_empty("assets/maps/somerset_chunks")) {
    m_chunksDir = "assets/maps/somerset_chunks";
    updateChunkStreaming(m_playerPos.x, m_playerPos.z);
    std::cout << "RoadTerrainViewer: Streaming 3x3 road chunks from "
              << m_chunksDir << std::endl;
  } else if (std::filesystem::exists("assets/maps/new_jersey_chunks") &&
             !std::filesystem::is_empty("assets/maps/new_jersey_chunks")) {
    m_chunksDir = "assets/maps/new_jersey_chunks";
    updateChunkStreaming(m_playerPos.x, m_playerPos.z);
    std::cout << "RoadTerrainViewer: Streaming 3x3 road chunks from "
              << m_chunksDir << std::endl;
  } else if (!loadAndDrapeRoads(roadGeoJsonPath)) {
    std::cerr << "RoadTerrainViewer: Failed to load and drape roads from "
              << roadGeoJsonPath << std::endl;
  }

  // Set initial mouse lock
  setMouseLocked(true);

  std::cout << "RoadTerrainViewer: Initialized successfully with "
            << m_roadSegments.size() << " road segments, "
            << m_roadJunctions.size() << " junctions, and "
            << m_terrainRenderer.getPack().header.tileCount << " terrain tiles."
            << std::endl;
  return true;
}

bool RoadTerrainViewer::loadTerrainPack(const std::string &packPath) {
  if (!m_terrainRenderer.loadPack(packPath)) {
    return false;
  }

  if (m_terrainRenderer.isQuadsphere()) {
    m_currentLat = 40.592272;
    m_currentLon = -74.522966;

    const double cLat = 40.592676;
    const double cLon = -74.515211;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

    m_playerPos.x = static_cast<float>((m_currentLon - cLon) * m_per_lon);
    m_playerPos.z = static_cast<float>(-(m_currentLat - cLat) * m_per_lat);

    float groundH = 105.0f;
    m_terrainRenderer.hasElevationGeodetic(m_currentLat, m_currentLon, groundH);
    m_groundElevation = groundH;
    m_currentAlt = m_groundElevation + m_eyeHeight;

    m_camPos = glm::vec3(m_playerPos.x, m_groundElevation + m_eyeHeight, m_playerPos.z);
    m_yaw = -90.0f;
    m_pitch = -8.0f;
    return true;
  }

  // Update GPS anchor info
  const auto &pack = m_terrainRenderer.getPack();
  float startE =
      pack.startEast + (pack.tilesX_LOD0 * pack.header.baseTileSize) * 0.5f;
  float startN =
      pack.startNorth + (pack.tilesY_LOD0 * pack.header.baseTileSize) * 0.5f;

  m_playerPos.x = startE;
  m_playerPos.z = -startN;
  m_terrainRenderer.prewarmDestinationTiles(m_playerPos.x, m_playerPos.z);
  m_groundElevation =
      m_terrainRenderer.sampleElevation(m_playerPos.x, m_playerPos.z);
  m_camPos =
      glm::vec3(m_playerPos.x, m_groundElevation + m_eyeHeight, m_playerPos.z);

  // Compute initial WGS84 Geodetic coordinates
  glm::dvec3 originECEF(pack.header.ecefAnchor[0], pack.header.ecefAnchor[1],
                        pack.header.ecefAnchor[2]);
  glm::vec3 enu(m_playerPos.x, -m_playerPos.z, m_camPos.y);
  glm::dvec3 ecef =
      TerrainLoader::enuToECEF(enu, originECEF, pack.header.enuRotation);
  glm::dvec3 geo = TerrainLoader::ecefToGeodetic(ecef);

  m_currentLat = geo.x;
  m_currentLon = geo.y;
  m_currentAlt = geo.z;

  return true;
}

float RoadTerrainViewer::sampleGroundElevation(float worldX, float worldZ) {
  if (m_terrainRenderer.isQuadsphere()) {
    const double cLat = 40.592676;
    const double cLon = -74.515211;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

    double lat = cLat - (static_cast<double>(worldZ) / m_per_lat);
    double lon = cLon + (static_cast<double>(worldX) / m_per_lon);
    return m_terrainRenderer.sampleElevationGeodetic(lat, lon);
  }
  return m_terrainRenderer.sampleElevation(worldX, worldZ);
}

bool RoadTerrainViewer::hasGroundElevation(float worldX, float worldZ,
                                           float &outElev) {
  if (m_terrainRenderer.isQuadsphere()) {
    const double cLat = 40.592676;
    const double cLon = -74.515211;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

    double lat = cLat - (static_cast<double>(worldZ) / m_per_lat);
    double lon = cLon + (static_cast<double>(worldX) / m_per_lon);
    return m_terrainRenderer.hasElevationGeodetic(lat, lon, outElev);
  }
  return m_terrainRenderer.hasElevation(worldX, worldZ, outElev);
}

bool RoadTerrainViewer::findRoadElevation(float worldX, float worldZ, float &outElev) const {
  bool found = false;
  float highestElev = -9999.0f;

  if (m_terrainRenderer.isQuadsphere()) {
    const double cLat = 40.592676;
    const double cLon = -74.515211;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

    double lat = cLat - (static_cast<double>(worldZ) / m_per_lat);
    double lon = cLon + (static_cast<double>(worldX) / m_per_lon);
    glm::dvec3 pECEF = Quadsphere::geodeticToECEF(lat, lon, 0.0);

    for (const auto &kv : m_loadedQuadMeshes) {
      for (const auto &seg : kv.second.segments) {
        if (seg.splinePoints.size() < 2) continue;
        float halfWidth = seg.totalWidth * 0.5f + 1.5f;
        double halfWidthSq = (double)halfWidth * (double)halfWidth;

        for (size_t i = 0; i + 1 < seg.splinePoints.size(); ++i) {
          const auto &sp0 = seg.splinePoints[i];
          const auto &sp1 = seg.splinePoints[i + 1];

          glm::dvec3 d = sp1.posECEF - sp0.posECEF;
          double lenSq = glm::dot(d, d);
          if (lenSq < 1e-4) continue;

          double t = std::clamp(glm::dot(pECEF - sp0.posECEF, d) / lenSq, 0.0, 1.0);
          glm::dvec3 proj = sp0.posECEF + t * d;
          double distSq = glm::dot(pECEF - proj, pECEF - proj);

          if (distSq <= halfWidthSq * 4.0) {
            float roadH = static_cast<float>(glm::length(proj) - Quadsphere::EARTH_RADIUS);
            if (roadH > highestElev) {
              highestElev = roadH;
              found = true;
            }
          }
        }
      }
    }
  }

  for (const auto &seg : m_roadSegments) {
    if (seg.splinePoints.size() < 2) continue;
    float halfWidth = seg.totalWidth * 0.5f + 1.5f; // with 1.5m curb/shoulder margin
    float halfWidthSq = halfWidth * halfWidth;

    for (size_t i = 0; i + 1 < seg.splinePoints.size(); ++i) {
      const auto &p0 = seg.splinePoints[i].pos;
      const auto &p1 = seg.splinePoints[i + 1].pos;

      float minX = std::min(p0.x, p1.x) - halfWidth;
      float maxX = std::max(p0.x, p1.x) + halfWidth;
      float minZ = std::min(p0.z, p1.z) - halfWidth;
      float maxZ = std::max(p0.z, p1.z) + halfWidth;
      if (worldX < minX || worldX > maxX || worldZ < minZ || worldZ > maxZ) {
        continue;
      }

      float dx = p1.x - p0.x;
      float dz = p1.z - p0.z;
      float lenSq = dx * dx + dz * dz;
      if (lenSq < 1e-4f) continue;

      float t = ((worldX - p0.x) * dx + (worldZ - p0.z) * dz) / lenSq;
      t = std::clamp(t, 0.0f, 1.0f);

      float projX = p0.x + t * dx;
      float projZ = p0.z + t * dz;
      float distSq = (worldX - projX) * (worldX - projX) + (worldZ - projZ) * (worldZ - projZ);

      if (distSq <= halfWidthSq) {
        float segElev = p0.y + t * (p1.y - p0.y);
        if (segElev > highestElev) {
          highestElev = segElev;
          found = true;
        }
      }
    }
  }

  if (found) {
    outElev = highestElev;
    return true;
  }
  return false;
}

bool RoadTerrainViewer::loadAndDrapeRoads(const std::string &geojsonPath) {
  m_roadGeoJsonPath = geojsonPath;
  m_rawRoadFeatures.clear();
  m_roadSegments.clear();
  m_roadJunctions.clear();
  m_mergedRoadMesh.reset();

  if (m_terrainRenderer.isPackLoaded() && !m_terrainRenderer.isQuadsphere()) {
    const auto &pack = m_terrainRenderer.getPack();
    GeoJsonLoader::loadWithAnchor(geojsonPath, pack.header.ecefAnchor,
                                  pack.header.enuRotation, m_rawRoadFeatures);
  } else {
    double cLon = -74.515211, cLat = 40.592676;
    GeoJsonLoader::load(geojsonPath, m_rawRoadFeatures, cLon, cLat);
  }

  if (m_rawRoadFeatures.empty()) {
    std::cerr << "RoadTerrainViewer: No road features found in " << geojsonPath
              << std::endl;
    return false;
  }

  // Process raw road features into spline centerlines
  RoadGenerator::processFeatures(m_rawRoadFeatures, 3.65f, 1.50f, 2.50f,
                                 m_roadSegments, false);

  m_totalRoadLengthMeters = 0.0f;
  m_elevatedBridgeCount = 0;
  for (const auto &seg : m_roadSegments) {
    m_totalRoadLengthMeters += seg.length;
    if (seg.isBridge || seg.layer > 0)
      m_elevatedBridgeCount++;
  }

  // Solve road elevations using physics simulation (tension, adhesion,
  // repulsion)
  if (m_terrainRenderer.isPackLoaded()) {
    RoadElevationSolverOptions solverOptions;
    solverOptions.minClearance = 5.5f;
    solverOptions.groundOffset = 0.15f;
    solverOptions.verbose = true;

    auto sampleTerrain = [this](float x, float z) -> float {
      return sampleGroundElevation(x, z);
    };

    RoadElevationSolver::solve(m_roadSegments, sampleTerrain, solverOptions);
  }

  // Build network topology & junctions
  RoadGenerator::buildNetworkTopology(m_roadSegments, m_roadJunctions);
  RoadGenerator::generateJunctionPatches(m_roadSegments, m_roadJunctions);

  // Generate ribbons with junction setbacks
  for (auto &s : m_roadSegments) {
    auto trimmed =
        RoadGenerator::trimSpline(s.splinePoints, s.startSetback, s.endSetback);
    std::vector<Vertex> verts;
    std::vector<uint32_t> idxs;
    RoadGenerator::generateRibbonGeometry(
        trimmed, s.totalWidth, s.lanes, s.oneway, s.shoulderWidth, verts, idxs);
    RoadGenerator::clipRibbonAgainstJunctions(verts, s, m_roadJunctions,
                                              m_roadSegments);
    if (!verts.empty() && !idxs.empty()) {
      s.visualMesh = std::make_shared<Mesh>(verts.data(), (Uint32)verts.size(),
                                            idxs.data(), (Uint32)idxs.size());
    }
  }

  // Build single unified GPU mesh for all roads and junction patches
  m_mergedRoadMesh =
      RoadGenerator::buildMergedMesh(m_roadSegments, m_roadJunctions, false);
  if (m_mergedRoadMesh) {
    m_totalRoadVertices = m_mergedRoadMesh->vertexCount;
    m_totalRoadTriangles = m_mergedRoadMesh->indexCount / 3;
  }

  // Teleport camera to first road segment start
  if (!m_roadSegments.empty() && !m_roadSegments.front().splinePoints.empty()) {
    const auto &startPt = m_roadSegments.front().splinePoints.front().pos;
    m_playerPos.x = startPt.x;
    m_playerPos.z = startPt.z;
    m_groundElevation = startPt.y;
    m_camPos = glm::vec3(startPt.x, startPt.y + m_eyeHeight, startPt.z);
    m_yaw = -90.0f;
    m_pitch = -8.0f;
  }

  return true;
}

void RoadTerrainViewer::updateChunkStreaming(float playerX, float playerZ) {
  if (m_terrainRenderer.isQuadsphere()) {
    uint8_t face = 0;
    double u = 0.0, v = 0.0;
    Quadsphere::geodeticToQuadsphere(m_currentLat, m_currentLon, face, u, v);
    double tilesPerSide = static_cast<double>(1ULL << 15);
    double span = 2.0 / tilesPerSide;
    int curTx = std::clamp(static_cast<int>(std::floor((u + 1.0) / span)), 0, (1 << 15) - 1);
    int curTy = std::clamp(static_cast<int>(std::floor((v + 1.0) / span)), 0, (1 << 15) - 1);

    if (curTx == m_lastPlayerQuadTx && curTy == m_lastPlayerQuadTy) {
      return;
    }
    m_lastPlayerQuadTx = curTx;
    m_lastPlayerQuadTy = curTy;

    const int streamRadius = (m_cameraMode == ViewerCameraMode::FlyDrone) ? 5 : 3;
    for (int dy = -streamRadius; dy <= streamRadius; ++dy) {
      for (int dx = -streamRadius; dx <= streamRadius; ++dx) {
        int tx = curTx + dx;
        int ty = curTy + dy;
        if (tx < 0 || ty < 0 || tx >= (1 << 15) || ty >= (1 << 15)) continue;

        std::tuple<uint8_t, uint32_t, uint32_t> key = { face, static_cast<uint32_t>(tx), static_cast<uint32_t>(ty) };
        if (m_loadedQuadMeshes.find(key) == m_loadedQuadMeshes.end()) {
          std::string path = m_chunksDir + "/chunk_" + std::to_string(face) + "_" +
                             std::to_string(tx) + "_" + std::to_string(ty) + ".froad";
          std::ifstream f(path);
          if (f.good()) {
            f.close();
            std::vector<ProcessedRoadSegment> segs;
            std::shared_ptr<Mesh> mesh;
            FRoadChunkHeader hdr;
            if (RoadBinary::loadChunk(path, segs, mesh, hdr, nullptr)) {
              glm::dvec3 center(hdr.chunkCenterECEF[0], hdr.chunkCenterECEF[1], hdr.chunkCenterECEF[2]);
              m_loadedQuadMeshes[key] = { center, mesh, std::move(segs) };
            } else {
              m_loadedQuadMeshes[key] = { glm::dvec3(0.0), nullptr, {} };
            }
          } else {
            m_loadedQuadMeshes[key] = { glm::dvec3(0.0), nullptr, {} };
          }
        }
      }
    }

    // Cache cleanup
    int maxDist = streamRadius + 3;
    if (m_loadedQuadMeshes.size() > 256) {
      for (auto it = m_loadedQuadMeshes.begin(); it != m_loadedQuadMeshes.end();) {
        int tx = static_cast<int>(std::get<1>(it->first));
        int ty = static_cast<int>(std::get<2>(it->first));
        int dist = std::max(std::abs(tx - curTx), std::abs(ty - curTy));
        if (dist > maxDist) {
          it = m_loadedQuadMeshes.erase(it);
        } else {
          ++it;
        }
      }
    }
    return;
  }

  int curChunkX = static_cast<int>(std::floor((playerX + 256.0f) / 512.0f));
  int curChunkZ = static_cast<int>(std::floor((playerZ + 256.0f) / 512.0f));

  if (curChunkX == m_lastPlayerChunkX && curChunkZ == m_lastPlayerChunkZ) {
    return;
  }

  m_lastPlayerChunkX = curChunkX;
  m_lastPlayerChunkZ = curChunkZ;

  // Load any missing chunks in window around player (7x7 on ground, 11x11
  // in FlyDrone cam)
  const int streamRadius = (m_cameraMode == ViewerCameraMode::FlyDrone) ? 5 : 3;
  for (int dz = -streamRadius; dz <= streamRadius; ++dz) {
    for (int dx = -streamRadius; dx <= streamRadius; ++dx) {
      int cx = curChunkX + dx;
      int cz = curChunkZ + dz;
      std::pair<int, int> coord = {cx, cz};
      if (m_loadedChunkMeshes.find(coord) == m_loadedChunkMeshes.end()) {
        std::string path = m_chunksDir + "/chunk_" + std::to_string(cx) + "_" +
                           std::to_string(cz) + ".froad";
        std::ifstream f(path);
        if (f.good()) {
          f.close();
          std::vector<ProcessedRoadSegment> segs;
          std::shared_ptr<Mesh> mesh;
          FRoadChunkHeader hdr;
          if (RoadBinary::loadChunk(path, segs, mesh, hdr, nullptr)) {
            m_loadedChunkMeshes[coord] = mesh;
          } else {
            m_loadedChunkMeshes[coord] = nullptr;
          }
        } else {
          m_loadedChunkMeshes[coord] = nullptr;
        }
      }
    }
  }

  // Cache cleanup: evict chunks further than streamRadius + 3
  int maxDist = streamRadius + 3;
  if (m_loadedChunkMeshes.size() > 256) {
    for (auto it = m_loadedChunkMeshes.begin();
         it != m_loadedChunkMeshes.end();) {
      int dist = std::max(std::abs(it->first.first - curChunkX),
                          std::abs(it->first.second - curChunkZ));
      if (dist > maxDist) {
        it = m_loadedChunkMeshes.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void RoadTerrainViewer::setMouseLocked(bool locked) {
  m_mouseLocked = locked;
  if (m_window) {
    SDL_SetWindowRelativeMouseMode(m_window, locked);
    if (locked) {
      SDL_HideCursor();
    } else {
      SDL_ShowCursor();
    }
  }
}

void RoadTerrainViewer::setCameraMode(ViewerCameraMode mode) {
  m_cameraMode = mode;
  m_lastPlayerChunkX = -999999;
  m_lastPlayerChunkZ = -999999;
  if (mode == ViewerCameraMode::FirstPersonGround) {
    float elev = m_groundElevation;
    if (hasGroundElevation(m_playerPos.x, m_playerPos.z, elev)) {
      m_groundElevation = elev;
    }
    float roadElev = 0.0f;
    if (findRoadElevation(m_playerPos.x, m_playerPos.z, roadElev)) {
      m_groundElevation = std::max(m_groundElevation, roadElev);
    }
    m_camPos.y = m_groundElevation + m_eyeHeight;
  }
}

bool RoadTerrainViewer::openCoordinateDialog(double &outLat, double &outLon) {
  std::string defaultCoord = "40.583188, -74.623174";
  if (std::abs(m_currentLat) > 0.001 && std::abs(m_currentLon) > 0.001) {
    std::ostringstream defSS;
    defSS << std::fixed << std::setprecision(6) << m_currentLat << ", "
          << m_currentLon;
    defaultCoord = defSS.str();
  }

  std::string script =
      "osascript -e 'set dlg to display dialog "
      "\"Enter Target Coordinates (Latitude, Longitude):\\nExample: 40.583188, "
      "-74.623174\" "
      "default answer \"" +
      defaultCoord +
      "\" "
      "with title \"Teleport to Coordinates\" "
      "buttons {\"Cancel\", \"Teleport\"} default button \"Teleport\"' "
      "-e 'text returned of dlg' 2>/dev/null";

  FILE *pipe = popen(script.c_str(), "r");
  if (!pipe)
    return false;

  char buffer[512];
  std::string result = "";
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }
  int rc = pclose(pipe);
  if (rc != 0 || result.empty())
    return false;

  while (!result.empty() && (result.back() == '\n' || result.back() == '\r' ||
                             result.back() == ' ')) {
    result.pop_back();
  }
  size_t start = result.find_first_not_of(" \t\r\n");
  if (start != std::string::npos) {
    result = result.substr(start);
  }

  for (char &c : result) {
    if (c == ',')
      c = ' ';
  }

  std::stringstream ss(result);
  double lat = 0.0, lon = 0.0;
  if (ss >> lat >> lon) {
    if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
      outLat = lat;
      outLon = lon;
      return true;
    }
  }
  std::cerr << "RoadTerrainViewer: Invalid coordinate input: '" << result << "'"
            << std::endl;
  return false;
}

void RoadTerrainViewer::promptAndTeleport() {
  bool wasLocked = m_mouseLocked;
  if (wasLocked) {
    setMouseLocked(false);
  }

  double lat = 0.0, lon = 0.0;
  if (openCoordinateDialog(lat, lon)) {
    float altOffset =
        (m_cameraMode == ViewerCameraMode::FirstPersonGround) ? 1.6f : 15.0f;
    teleportToCoordinates(lat, lon, altOffset);
  }

  if (wasLocked) {
    setMouseLocked(false);
  }
}

bool RoadTerrainViewer::teleportToCoordinates(double targetLat,
                                              double targetLon,
                                              float altitudeOffset) {
  if (!m_terrainRenderer.isPackLoaded())
    return false;

  if (m_terrainRenderer.isQuadsphere()) {
    const double cLat = 40.592676;
    const double cLon = -74.515211;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

    float posX = static_cast<float>((targetLon - cLon) * m_per_lon);
    float posZ = -static_cast<float>((targetLat - cLat) * m_per_lat);

    float groundElev = 30.0f;
    m_terrainRenderer.hasElevationGeodetic(targetLat, targetLon, groundElev);
    m_groundElevation = groundElev;
    m_currentLat = targetLat;
    m_currentLon = targetLon;
    m_currentAlt = groundElev + altitudeOffset;

    m_camPos.x = posX;
    m_camPos.z = posZ;
    m_camPos.y = groundElev + altitudeOffset;
    m_playerPos.x = posX;
    m_playerPos.z = posZ;

    m_lastPlayerChunkX = -999999;
    m_lastPlayerChunkZ = -999999;
    updateChunkStreaming(posX, posZ);

    std::cout << "RoadTerrainViewer: Teleported to Quadsphere GPS ("
              << targetLat << " N, " << targetLon << " W) "
              << "Elev: " << groundElev << "m" << std::endl;
    return true;
  }

  const auto &pack = m_terrainRenderer.getPack();
  glm::dvec3 ecef = TerrainLoader::geodeticToECEF(targetLat, targetLon, 0.0);
  glm::dvec3 ecefAnchor(pack.header.ecefAnchor[0], pack.header.ecefAnchor[1],
                        pack.header.ecefAnchor[2]);
  glm::vec3 enu =
      TerrainLoader::ecefToENU(ecef, ecefAnchor, pack.header.enuRotation);

  float posX = enu.x;
  float posZ = -enu.y;

  m_terrainRenderer.prewarmDestinationTiles(posX, posZ);
  float groundElev = 0.0f;
  hasGroundElevation(posX, posZ, groundElev);

  m_playerPos.x = posX;
  m_playerPos.z = posZ;
  m_groundElevation = groundElev;

  m_camPos.x = posX;
  m_camPos.z = posZ;
  m_camPos.y = groundElev + altitudeOffset;

  m_currentLat = targetLat;
  m_currentLon = targetLon;
  m_currentAlt = groundElev + altitudeOffset;

  m_lastPlayerChunkX = -999999;
  m_lastPlayerChunkZ = -999999;
  updateChunkStreaming(posX, posZ);

  std::cout << "RoadTerrainViewer: Teleported to (" << targetLat << " N, "
            << targetLon << " W) "
            << "ENU: (" << posX << ", " << posZ << ") Elev: " << groundElev
            << "m" << std::endl;
  return true;
}

void RoadTerrainViewer::updateCamera(float dt) {
  float radYaw = glm::radians(m_yaw);
  float radPitch = glm::radians(m_pitch);
  glm::vec3 forward;
  forward.x = std::cos(radYaw) * std::cos(radPitch);
  forward.y = std::sin(radPitch);
  forward.z = std::sin(radYaw) * std::cos(radPitch);
  forward = glm::normalize(forward);

  glm::vec3 planarForward =
      glm::normalize(glm::vec3(std::cos(radYaw), 0.0f, std::sin(radYaw)));
  glm::vec3 right =
      glm::normalize(glm::cross(planarForward, glm::vec3(0.0f, 1.0f, 0.0f)));

  if (m_cameraMode == ViewerCameraMode::FirstPersonGround) {
    float speed = m_keyShift ? m_sprintSpeed : m_walkSpeed;
    m_currentSpeed = 0.0f;

    glm::vec3 moveDir(0.0f);
    if (m_keyW)
      moveDir += planarForward;
    if (m_keyS)
      moveDir -= planarForward;
    if (m_keyA)
      moveDir -= right;
    if (m_keyD)
      moveDir += right;

    if (glm::length(moveDir) > 0.001f) {
      moveDir = glm::normalize(moveDir);
      m_playerPos += moveDir * speed * dt;
      m_currentSpeed = speed;
    }

    float elev = 0.0f;
    if (hasGroundElevation(m_playerPos.x, m_playerPos.z, elev)) {
      m_groundElevation = elev;
    }
    float roadElev = 0.0f;
    if (findRoadElevation(m_playerPos.x, m_playerPos.z, roadElev)) {
      m_groundElevation = std::max(m_groundElevation, roadElev);
    }
    m_camPos.x = m_playerPos.x;
    m_camPos.z = m_playerPos.z;
    m_camPos.y = m_groundElevation + m_eyeHeight;
  } else {
    float speed = m_keyShift ? m_droneFastSpeed : m_droneSpeed;
    m_currentSpeed = 0.0f;

    glm::vec3 moveDir(0.0f);
    if (m_keyW)
      moveDir += forward;
    if (m_keyS)
      moveDir -= forward;
    if (m_keyA)
      moveDir -= right;
    if (m_keyD)
      moveDir += right;
    if (m_keyE || m_keySpace)
      moveDir += glm::vec3(0.0f, 1.0f, 0.0f);
    if (m_keyQ || m_keyCtrl)
      moveDir -= glm::vec3(0.0f, 1.0f, 0.0f);

    if (glm::length(moveDir) > 0.001f) {
      moveDir = glm::normalize(moveDir);
      m_camPos += moveDir * speed * dt;
      m_playerPos.x = m_camPos.x;
      m_playerPos.z = m_camPos.z;
      m_currentSpeed = speed;
    }

    float elev = 0.0f;
    if (hasGroundElevation(m_camPos.x, m_camPos.z, elev)) {
      m_groundElevation = elev;
    }
    float roadElev = 0.0f;
    if (findRoadElevation(m_camPos.x, m_camPos.z, roadElev)) {
      m_groundElevation = std::max(m_groundElevation, roadElev);
    }
    if (m_camPos.y < m_groundElevation + 0.5f) {
      m_camPos.y = m_groundElevation + 0.5f;
    }
  }

  // Update real-time GPS telemetry
  if (m_terrainRenderer.isPackLoaded()) {
    if (m_terrainRenderer.isQuadsphere()) {
      const double cLat = 40.592676;
      const double cLon = -74.515211;
      const double degToRad = 3.14159265358979323846 / 180.0;
      const double m_per_lat = 111139.0;
      const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

      m_currentLat = cLat - (static_cast<double>(m_camPos.z) / m_per_lat);
      m_currentLon = cLon + (static_cast<double>(m_camPos.x) / m_per_lon);
      m_currentAlt = m_camPos.y;
    } else {
      const auto &pack = m_terrainRenderer.getPack();
      glm::dvec3 originECEF(pack.header.ecefAnchor[0],
                            pack.header.ecefAnchor[1],
                            pack.header.ecefAnchor[2]);
      glm::vec3 enu(m_camPos.x, -m_camPos.z, m_camPos.y);
      glm::dvec3 ecef =
          TerrainLoader::enuToECEF(enu, originECEF, pack.header.enuRotation);
      glm::dvec3 geo = TerrainLoader::ecefToGeodetic(ecef);

      m_currentLat = geo.x;
      m_currentLon = geo.y;
      m_currentAlt = geo.z;
    }
  }
}

glm::mat4 RoadTerrainViewer::getViewMatrix() const {
  glm::vec3 eye = m_camPos;
  glm::vec3 forward = getCameraForward();
  return glm::lookAt(eye, eye + forward, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 RoadTerrainViewer::getProjMatrix(float aspect) const {
  return glm::perspective(glm::radians(60.0f), aspect, 0.2f, 20000.0f);
}

glm::vec3 RoadTerrainViewer::getCameraPosition() const { return m_camPos; }

glm::vec3 RoadTerrainViewer::getCameraForward() const {
  float radYaw = glm::radians(m_yaw);
  float radPitch = glm::radians(m_pitch);
  glm::vec3 forward;
  forward.x = std::cos(radYaw) * std::cos(radPitch);
  forward.y = std::sin(radPitch);
  forward.z = std::sin(radYaw) * std::cos(radPitch);
  return glm::normalize(forward);
}

void RoadTerrainViewer::handleEvent(const SDL_Event &event) {
  if (event.type == SDL_EVENT_KEY_DOWN) {
    switch (event.key.scancode) {
    case SDL_SCANCODE_W:
      m_keyW = true;
      break;
    case SDL_SCANCODE_S:
      m_keyS = true;
      break;
    case SDL_SCANCODE_A:
      m_keyA = true;
      break;
    case SDL_SCANCODE_D:
      m_keyD = true;
      break;
    case SDL_SCANCODE_Q:
      m_keyQ = true;
      break;
    case SDL_SCANCODE_E:
      m_keyE = true;
      break;
    case SDL_SCANCODE_SPACE:
      m_keySpace = true;
      break;
    case SDL_SCANCODE_LSHIFT:
    case SDL_SCANCODE_RSHIFT:
      m_keyShift = true;
      break;
    case SDL_SCANCODE_LCTRL:
    case SDL_SCANCODE_RCTRL:
      m_keyCtrl = true;
      break;

    case SDL_SCANCODE_TAB:
      setMouseLocked(!m_mouseLocked);
      break;
    case SDL_SCANCODE_V:
      if (m_cameraMode == ViewerCameraMode::FirstPersonGround) {
        setCameraMode(ViewerCameraMode::FlyDrone);
        m_camPos.y += 20.0f;
      } else {
        setCameraMode(ViewerCameraMode::FirstPersonGround);
      }
      break;
    case SDL_SCANCODE_B:
      m_renderRoads = !m_renderRoads;
      break;
    case SDL_SCANCODE_T:
      m_renderTerrain = !m_renderTerrain;
      break;
    case SDL_SCANCODE_L:
      m_terrainRenderer.setLodColorTint(!m_terrainRenderer.getLodColorTint());
      break;
    case SDL_SCANCODE_C:
      m_terrainRenderer.setEarthCurvature(
          !m_terrainRenderer.getEarthCurvature());
      break;
    case SDL_SCANCODE_F:
      m_terrainRenderer.setFrustumCulling(
          !m_terrainRenderer.getFrustumCulling());
      break;
    case SDL_SCANCODE_1:
      if (std::ifstream("jughandle/somerset_mc_lerc.fterrainpack").good()) {
        loadTerrainPack("jughandle/somerset_mc_lerc.fterrainpack");
      }
      break;
    case SDL_SCANCODE_R:
      if (!m_roadSegments.empty() &&
          !m_roadSegments.front().splinePoints.empty()) {
        const auto &p = m_roadSegments.front().splinePoints.front().pos;
        m_playerPos = p;
        m_groundElevation = p.y;
        m_camPos = glm::vec3(p.x, p.y + m_eyeHeight, p.z);
        m_yaw = -90.0f;
        m_pitch = -8.0f;
      }
      break;
    default:
      break;
    }
  } else if (event.type == SDL_EVENT_KEY_UP) {
    switch (event.key.scancode) {
    case SDL_SCANCODE_W:
      m_keyW = false;
      break;
    case SDL_SCANCODE_S:
      m_keyS = false;
      break;
    case SDL_SCANCODE_A:
      m_keyA = false;
      break;
    case SDL_SCANCODE_D:
      m_keyD = false;
      break;
    case SDL_SCANCODE_Q:
      m_keyQ = false;
      break;
    case SDL_SCANCODE_E:
      m_keyE = false;
      break;
    case SDL_SCANCODE_SPACE:
      m_keySpace = false;
      break;
    case SDL_SCANCODE_LSHIFT:
    case SDL_SCANCODE_RSHIFT:
      m_keyShift = false;
      break;
    case SDL_SCANCODE_LCTRL:
    case SDL_SCANCODE_RCTRL:
      m_keyCtrl = false;
      break;
    default:
      break;
    }
  } else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
    if (event.button.button == SDL_BUTTON_RIGHT)
      m_mouseRightDown = true;
    if (event.button.button == SDL_BUTTON_LEFT) {
      m_mouseLeftDown = true;
      m_mouseLeftClicked = true;
    }
  } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
    if (event.button.button == SDL_BUTTON_RIGHT)
      m_mouseRightDown = false;
    if (event.button.button == SDL_BUTTON_LEFT)
      m_mouseLeftDown = false;
  } else if (event.type == SDL_EVENT_MOUSE_MOTION) {
    float dx = event.motion.xrel;
    float dy = event.motion.yrel;
    m_mouseX = event.motion.x;
    m_mouseY = event.motion.y;

    if (m_mouseLocked) {
      m_yaw += dx * 0.16f;
      m_pitch -= dy * 0.16f;
      m_pitch = std::clamp(m_pitch, -85.0f, 85.0f);
    } else if (m_mouseRightDown && !UI::wantsMouseInput()) {
      m_yaw += dx * 0.16f;
      m_pitch -= dy * 0.16f;
      m_pitch = std::clamp(m_pitch, -85.0f, 85.0f);
    }
  } else if (event.type == SDL_EVENT_MOUSE_WHEEL) {
    m_mouseWheel += event.wheel.y;
  } else if (event.type == SDL_EVENT_WINDOW_MOUSE_ENTER ||
             event.type == SDL_EVENT_WINDOW_FOCUS_GAINED) {
    if (m_mouseLocked) {
      SDL_SetWindowRelativeMouseMode(m_window, true);
      SDL_HideCursor();
    }
  }
}

void RoadTerrainViewer::update(float dt) {
  updateCamera(dt);
  updateChunkStreaming(m_playerPos.x, m_playerPos.z);

  float aspect = 16.0f / 9.0f;
  if (m_window) {
    int pixW = 0, pixH = 0;
    SDL_GetWindowSizeInPixels(m_window, &pixW, &pixH);
    if (pixW > 0 && pixH > 0)
      aspect = static_cast<float>(pixW) / static_cast<float>(pixH);
  }
  if (m_terrainRenderer.isQuadsphere()) {
    glm::dvec3 camPosECEF =
        Quadsphere::geodeticToECEF(m_currentLat, m_currentLon, m_currentAlt);
    m_terrainRenderer.update(getCameraPosition(), getCameraForward(),
                             getViewMatrix(), getProjMatrix(aspect), aspect, dt,
                             &camPosECEF);
  } else {
    m_terrainRenderer.update(getCameraPosition(), getCameraForward(),
                             getViewMatrix(), getProjMatrix(aspect), aspect,
                             dt);
  }
}

void RoadTerrainViewer::render(Engine &engine) {
  auto &rhi = RHI_Device::getInstance();
  int logW = 0, logH = 0;
  SDL_GetWindowSize(engine.getWindow(), &logW, &logH);
  int pixW = 0, pixH = 0;
  SDL_GetWindowSizeInPixels(engine.getWindow(), &pixW, &pixH);
  if (logW <= 0 || logH <= 0 || pixW <= 0 || pixH <= 0)
    return;

  float aspect = static_cast<float>(pixW) / static_cast<float>(pixH);

  SDL_GPUTexture *swapchainTexture = nullptr;
  SDL_GPUCommandBuffer *cmdBuf = rhi.beginFrame(swapchainTexture);
  if (!cmdBuf || !swapchainTexture)
    return;

  // 1. Create Transient Depth Attachment
  SDL_GPUTextureCreateInfo depthInfo = {};
  depthInfo.type = SDL_GPU_TEXTURETYPE_2D;
  depthInfo.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
  depthInfo.width = pixW;
  depthInfo.height = pixH;
  depthInfo.layer_count_or_depth = 1;
  depthInfo.num_levels = 1;
  depthInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
  depthInfo.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

  SDL_GPUTexture *depthTexture =
      SDL_CreateGPUTexture(rhi.getDevice(), &depthInfo);

  // 2. PASS 1: 3D Scene Pass (Terrain + Roads)
  SDL_GPUColorTargetInfo colorTarget = {};
  colorTarget.texture = swapchainTexture;
  colorTarget.clear_color = {0.45f, 0.68f, 0.92f, 1.0f}; // Sky Blue Background
  colorTarget.load_op = SDL_GPU_LOADOP_CLEAR;
  colorTarget.store_op = SDL_GPU_STOREOP_STORE;

  SDL_GPUDepthStencilTargetInfo depthTarget = {};
  depthTarget.texture = depthTexture;
  depthTarget.cycle = false;
  depthTarget.clear_depth = 1.0f;
  depthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
  depthTarget.store_op = SDL_GPU_STOREOP_DONT_CARE;

  SDL_GPURenderPass *pass =
      SDL_BeginGPURenderPass(cmdBuf, &colorTarget, 1, &depthTarget);
  if (pass) {
    glm::mat4 view = getViewMatrix();
    glm::mat4 proj = getProjMatrix(aspect);

    // A. Draw Hierarchical Quadtree Terrain
    if (m_renderTerrain) {
      if (m_terrainRenderer.isQuadsphere()) {
        glm::dvec3 camPosECEF = Quadsphere::geodeticToECEF(
            m_currentLat, m_currentLon, m_currentAlt);
        m_terrainRenderer.render(pass, cmdBuf, view, proj, getCameraPosition(),
                                 &camPosECEF);
      } else {
        m_terrainRenderer.render(pass, cmdBuf, view, proj, getCameraPosition());
      }
    }

    // B. Draw Vector Road Network via shared RoadRenderer (streamed area around
    // player)
    if (m_renderRoads) {
      if (m_terrainRenderer.isQuadsphere()) {
        glm::dvec3 camPosECEF = Quadsphere::geodeticToECEF(
            m_currentLat, m_currentLon, m_currentAlt);
        double camLat, camLon, camAlt;
        Quadsphere::ecefToGeodetic(camPosECEF, camLat, camLon, camAlt);
        glm::mat4 R_ecef_to_enu =
            glm::mat4(Quadsphere::getECEFtoENU(camLat, camLon));

        glm::mat4 viewRot = view * R_ecef_to_enu;
        viewRot[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

        glm::vec3 sunECEF = glm::normalize(
            glm::vec3(glm::dmat3(Quadsphere::getENUtoECEF(camLat, camLon)) *
                      glm::dvec3(0.42, 0.82, -0.38)));

        for (const auto& kv : m_loadedQuadMeshes) {
          const auto& chunk = kv.second;
          if (chunk.mesh && chunk.mesh->indexCount > 0) {
            m_roadRenderer.drawMeshQuadsphere(
                pass, cmdBuf, viewRot, proj,
                camPosECEF, chunk.chunkCenterECEF,
                *chunk.mesh,
                glm::vec4(2.0f, 0.0f, 0.10f, 0.0f),
                sunECEF);
          }
        }
      } else {
        int curChunkX =
            static_cast<int>(std::floor((m_playerPos.x + 256.0f) / 512.0f));
        int curChunkZ =
            static_cast<int>(std::floor((m_playerPos.z + 256.0f) / 512.0f));

        const int renderRadius =
            (m_cameraMode == ViewerCameraMode::FlyDrone) ? 5 : 3;
        bool drewAny = false;
        for (int dz = -renderRadius; dz <= renderRadius; ++dz) {
          for (int dx = -renderRadius; dx <= renderRadius; ++dx) {
            auto it = m_loadedChunkMeshes.find({curChunkX + dx, curChunkZ + dz});
            if (it != m_loadedChunkMeshes.end() && it->second &&
                it->second->indexCount > 0) {
              m_roadRenderer.drawMesh(pass, cmdBuf, view, proj,
                                      getCameraPosition(), *it->second,
                                      glm::vec4(2.0f, 0.0f, 0.10f, 0.0f));
              drewAny = true;
            }
          }
        }

        // Fallback for single loaded road mesh if chunks aren't used
        if (!drewAny && m_mergedRoadMesh && m_mergedRoadMesh->indexCount > 0) {
          m_roadRenderer.drawMesh(pass, cmdBuf, view, proj, getCameraPosition(),
                                  *m_mergedRoadMesh,
                                  glm::vec4(2.0f, 0.0f, 0.10f, 0.0f));
        }
      }
    }

    SDL_EndGPURenderPass(pass);
  }

  SDL_ReleaseGPUTexture(rhi.getDevice(), depthTexture);

  // 3. PASS 2: UI Pass
  float uiMouseX = m_mouseLocked ? -1000.0f : m_mouseX;
  float uiMouseY = m_mouseLocked ? -1000.0f : m_mouseY;
  bool uiMouseDown = m_mouseLocked ? false : m_mouseLeftDown;
  bool uiMouseClicked = m_mouseLocked ? false : m_mouseLeftClicked;

  UI::beginFrame(static_cast<float>(logW), static_cast<float>(logH), uiMouseX,
                 uiMouseY, uiMouseDown, uiMouseClicked, m_mouseWheel);
  m_mouseLeftClicked = false;
  m_mouseWheel = 0.0f;

  renderUI(logW, logH);

  UI::endFrame();
  UI::render(cmdBuf, swapchainTexture);

  // 4. Submit Frame
  rhi.endFrame(cmdBuf);
}

void RoadTerrainViewer::renderUI(int screenW, int screenH) {
  float fScreenW = static_cast<float>(screenW);
  float fScreenH = static_cast<float>(screenH);
  m_uiScale = std::max(0.75f, std::min(fScreenW / 1440.0f, fScreenH / 900.0f));

  // 1. Top Header Bar
  float headerH = 38.0f * m_uiScale;
  UI::panel("", 0.0f, 0.0f, fScreenW, headerH,
            glm::vec4(0.08f, 0.10f, 0.14f, 0.96f));
  UI::label("ForceEngine - Road & Terrain Viewer",
            16.0f * m_uiScale, 9.0f * m_uiScale, 1.10f * m_uiScale,
            glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

  // Compass HUD (Top Screen Center)
  glm::vec3 camFwd = getCameraForward();
  float headingRad = std::atan2(camFwd.x, -camFwd.z);
  float headingDeg = glm::degrees(headingRad);
  if (headingDeg < 0.0f) headingDeg += 360.0f;

  const std::string dirNames[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int sector = static_cast<int>(std::floor((headingDeg + 22.5f) / 45.0f)) % 8;
  if (sector < 0) sector += 8;

  const std::string &closestDir = dirNames[sector];
  const std::string &prevDir = dirNames[(sector + 7) % 8];
  const std::string &nextDir = dirNames[(sector + 1) % 8];

  float compassW = 280.0f * m_uiScale;
  float compassH = 26.0f * m_uiScale;
  float compassX = (fScreenW - compassW) * 0.5f;
  float compassY = 6.0f * m_uiScale;

  // Background & border
  UI::drawRect(compassX, compassY, compassW, compassH,
               glm::vec4(0.04f, 0.06f, 0.09f, 0.95f));
  UI::drawRectOutline(compassX, compassY, compassW, compassH, 1.0f,
                      glm::vec4(0.20f, 0.38f, 0.55f, 0.85f));

  float textY = compassY + 7.0f * m_uiScale;
  float curCompX = compassX + 10.0f * m_uiScale;

  UI::drawText("COMPASS", curCompX, textY, 0.90f * m_uiScale,
               glm::vec4(0.45f, 0.70f, 0.95f, 1.0f));
  curCompX += 68.0f * m_uiScale;

  // Neighbor prev direction
  UI::drawText(prevDir, curCompX, textY, 0.85f * m_uiScale,
               glm::vec4(0.50f, 0.55f, 0.65f, 0.80f));
  curCompX += 34.0f * m_uiScale;

  // Center active closest direction badge
  float badgeW = 52.0f * m_uiScale;
  float badgeH = 20.0f * m_uiScale;
  float badgeY = compassY + 3.0f * m_uiScale;
  UI::drawRect(curCompX, badgeY, badgeW, badgeH,
               glm::vec4(0.08f, 0.28f, 0.18f, 0.95f));
  UI::drawRectOutline(curCompX, badgeY, badgeW, badgeH, 1.0f,
                      glm::vec4(0.25f, 1.0f, 0.55f, 0.95f));

  // Small top indicator tick mark
  UI::drawRect(curCompX + badgeW * 0.5f - 1.0f, compassY, 2.0f, 3.0f * m_uiScale,
               glm::vec4(0.25f, 1.0f, 0.55f, 1.0f));

  float dirTextW = static_cast<float>(closestDir.length()) * 8.0f * 1.05f * m_uiScale;
  float dirTextX = curCompX + (badgeW - dirTextW) * 0.5f;
  UI::drawText(closestDir, dirTextX, textY, 1.05f * m_uiScale,
               glm::vec4(0.35f, 1.0f, 0.65f, 1.0f));
  curCompX += badgeW + 12.0f * m_uiScale;

  // Neighbor next direction
  UI::drawText(nextDir, curCompX, textY, 0.85f * m_uiScale,
               glm::vec4(0.50f, 0.55f, 0.65f, 0.80f));
  curCompX += 34.0f * m_uiScale;

  // Exact heading degrees
  std::stringstream ssDeg;
  ssDeg << std::setw(3) << std::setfill('0') << static_cast<int>(std::round(headingDeg)) << " deg";
  UI::drawText(ssDeg.str(), curCompX, textY, 0.88f * m_uiScale,
               glm::vec4(0.95f, 0.90f, 0.45f, 1.0f));

  // Mouse Lock Button
  float lockBtnW = 185.0f * m_uiScale;
  float lockBtnH = 24.0f * m_uiScale;
  float lockBtnX = fScreenW - lockBtnW - 12.0f * m_uiScale;
  std::string lockLabel =
      m_mouseLocked ? "[Tab] Mouse: LOCKED" : "[Tab] Mouse: UNLOCKED";
  if (UI::button(lockLabel, lockBtnX, 7.0f * m_uiScale, lockBtnW, lockBtnH,
                 m_mouseLocked)) {
    setMouseLocked(!m_mouseLocked);
  }

  // Camera Mode Toggle Button
  float camBtnW = 195.0f * m_uiScale;
  float camBtnH = 24.0f * m_uiScale;
  float camBtnX = lockBtnX - camBtnW - 8.0f * m_uiScale;
  std::string camLabel = (m_cameraMode == ViewerCameraMode::FirstPersonGround)
                             ? "[V] Ground (1.6m)"
                             : "[V] Drone (Fly-Cam)";
  if (UI::button(camLabel, camBtnX, 7.0f * m_uiScale, camBtnW, camBtnH,
                 m_cameraMode == ViewerCameraMode::FlyDrone)) {
    setCameraMode(m_cameraMode == ViewerCameraMode::FirstPersonGround
                      ? ViewerCameraMode::FlyDrone
                      : ViewerCameraMode::FirstPersonGround);
  }

  // 2. Bottom Controls Bar
  float bottomH = 26.0f * m_uiScale;
  float bottomY = fScreenH - bottomH;
  UI::panel("", 0.0f, bottomY, fScreenW, bottomH,
            glm::vec4(0.08f, 0.09f, 0.12f, 0.98f));
  UI::label("CONTROLS:  [W,A,S,D] Move  |  [Shift] Sprint  |  [V] Toggle "
            "Ground / Fly Drone  |  [Tab] Mouse Lock  |  [B] Toggle Roads  |  "
            "[T] Toggle Terrain  |  [R] Reset Origin  |  [1/2] LERC Datasets",
            14.0f * m_uiScale, bottomY + 5.0f * m_uiScale, 0.82f * m_uiScale,
            glm::vec4(0.80f, 0.84f, 0.90f, 1.0f));

  float contentTop = headerH + 10.0f * m_uiScale;
  float availableH = bottomY - contentTop - 10.0f * m_uiScale;

  // 3. Left Panel: Real-Time GPS & Teleport
  float panelW = 345.0f * m_uiScale;
  float panelH = std::min(availableH, 530.0f * m_uiScale);
  float panelX = 14.0f * m_uiScale;
  float panelY = contentTop;

  UI::panel("REAL-TIME GPS & TELEPORT", panelX, panelY, panelW, panelH,
            glm::vec4(0.10f, 0.11f, 0.14f, 0.94f));

  float curY = panelY + 30.0f * m_uiScale;
  float lineH = 16.0f * m_uiScale;

  // Unified Terrain Data Source
  UI::label("Terrain Data Source:", panelX + 10.0f * m_uiScale, curY,
            0.82f * m_uiScale, glm::vec4(0.4f, 0.7f, 1.0f, 1.0f));
  curY += lineH * 0.95f;
  std::string packTitle = "LERC Terrain Pack";
  if (m_terrainRenderer.getPack().header.tileCount > 5000000) {
    packTitle = "New Jersey Statewide 0.5m LiDAR (8.59M tiles)";
  } else if (m_terrainRenderer.getPackPath().find("somerset") !=
             std::string::npos) {
    packTitle = "Somerset Master Pack (16.4km LERC)";
  }
  UI::label(packTitle, panelX + 10.0f * m_uiScale, curY, 0.84f * m_uiScale,
            glm::vec4(0.85f, 0.88f, 0.92f, 1.0f));
  curY += 20.0f * m_uiScale;

  UI::separator(panelX + 8.0f * m_uiScale, curY, panelW - 16.0f * m_uiScale);
  curY += 8.0f * m_uiScale;

  // GPS Telemetry
  std::stringstream ss;
  ss << std::fixed << std::setprecision(6);
  UI::label("-- Real-Life Earth Coordinates (WGS84) --",
            panelX + 10.0f * m_uiScale, curY, 0.82f * m_uiScale,
            glm::vec4(0.4f, 0.7f, 1.0f, 1.0f));
  curY += lineH;

  ss.str("");
  ss << "Latitude:  " << std::abs(m_currentLat)
     << (m_currentLat >= 0 ? " N" : " S");
  UI::label(ss.str(), panelX + 10.0f * m_uiScale, curY, 0.88f * m_uiScale,
            glm::vec4(0.3f, 0.95f, 0.4f, 1.0f));
  curY += lineH;

  ss.str("");
  ss << "Longitude: " << std::abs(m_currentLon)
     << (m_currentLon >= 0 ? " E" : " W");
  UI::label(ss.str(), panelX + 10.0f * m_uiScale, curY, 0.88f * m_uiScale,
            glm::vec4(0.3f, 0.95f, 0.4f, 1.0f));
  curY += lineH;

  ss.str("");
  if (m_terrainRenderer.isQuadsphere()) {
    uint8_t face = 0;
    double u = 0.0, v = 0.0;
    Quadsphere::geodeticToQuadsphere(m_currentLat, m_currentLon, face, u, v);
    double tilesPerSide = static_cast<double>(1ULL << 15);
    double span = 2.0 / tilesPerSide;
    int tx = std::clamp(static_cast<int>(std::floor((u + 1.0) / span)), 0, (1 << 15) - 1);
    int ty = std::clamp(static_cast<int>(std::floor((v + 1.0) / span)), 0, (1 << 15) - 1);
    ss << "Chunk (L15): Face " << (int)face << " [" << tx << ", " << ty << "]";
  } else {
    int curChunkX =
        static_cast<int>(std::floor((m_playerPos.x + 256.0f) / 512.0f));
    int curChunkZ =
        static_cast<int>(std::floor((m_playerPos.z + 256.0f) / 512.0f));
    ss << "Chunk Coords: [" << curChunkX << ", " << curChunkZ << "] (512m)";
  }
  UI::label(ss.str(), panelX + 10.0f * m_uiScale, curY, 0.88f * m_uiScale,
            glm::vec4(0.25f, 0.85f, 1.0f, 1.0f));
  curY += lineH;

  ss.str("");
  ss << "Ground Elev: " << std::fixed << std::setprecision(1)
     << m_groundElevation << "m | Cam Y: " << m_camPos.y << "m";
  UI::label(ss.str(), panelX + 10.0f * m_uiScale, curY, 0.84f * m_uiScale,
            glm::vec4(0.75f, 0.80f, 0.85f, 1.0f));
  curY += lineH;

  ss.str("");
  ss << "Speed: " << std::fixed << std::setprecision(1) << m_currentSpeed
     << " m/s | Heading: " << std::fixed << std::setprecision(0) << headingDeg
     << " deg (" << closestDir << ")";
  UI::label(ss.str(), panelX + 10.0f * m_uiScale, curY, 0.84f * m_uiScale,
            glm::vec4(0.75f, 0.80f, 0.85f, 1.0f));
  curY += lineH;

  UI::separator(panelX + 8.0f * m_uiScale, curY, panelW - 16.0f * m_uiScale);
  curY += 8.0f * m_uiScale;

  // Navigation & Teleportation
  UI::label("Navigation & Teleportation:", panelX + 10.0f * m_uiScale, curY,
            0.80f * m_uiScale, glm::vec4(0.78f, 0.82f, 0.88f, 1.0f));
  curY += lineH * 0.95f;

  if (UI::button("Teleport (Lat, Lon)...", panelX + 10.0f * m_uiScale, curY,
                 panelW - 20.0f * m_uiScale, 22.0f * m_uiScale)) {
    promptAndTeleport();
  }
  curY += 26.0f * m_uiScale;

  UI::separator(panelX + 8.0f * m_uiScale, curY, panelW - 16.0f * m_uiScale);
  curY += 8.0f * m_uiScale;

  // Display / Render Layer Toggles
  UI::label("Rendering Layers & Shading:", panelX + 10.0f * m_uiScale, curY,
            0.82f * m_uiScale, glm::vec4(0.4f, 0.7f, 1.0f, 1.0f));
  curY += lineH;

  UI::toggle("Show Vector Roads [B]", m_renderRoads, panelX + 10.0f * m_uiScale,
             curY, panelW - 20.0f * m_uiScale, 18.0f * m_uiScale);
  curY += lineH * 1.35f;
  UI::toggle("Show Quadtree Terrain [T]", m_renderTerrain,
             panelX + 10.0f * m_uiScale, curY, panelW - 20.0f * m_uiScale,
             18.0f * m_uiScale);
  curY += lineH * 1.35f;

  bool lodTint = m_terrainRenderer.getLodColorTint();
  if (UI::toggle("Terrain LOD Tint [L]", lodTint, panelX + 10.0f * m_uiScale,
                 curY, panelW - 20.0f * m_uiScale, 18.0f * m_uiScale)) {
    m_terrainRenderer.setLodColorTint(lodTint);
  }
  curY += lineH * 1.35f;

  bool frustum = m_terrainRenderer.getFrustumCulling();
  if (UI::toggle("Frustum Culling [F]", frustum, panelX + 10.0f * m_uiScale,
                 curY, panelW - 20.0f * m_uiScale, 18.0f * m_uiScale)) {
    m_terrainRenderer.setFrustumCulling(frustum);
  }
  curY += lineH * 1.35f;

  // 4. Right Panel: Road & Quadtree Metrics
  float rPanelW = 345.0f * m_uiScale;
  float rPanelH = std::min(availableH, 500.0f * m_uiScale);
  float rPanelX = fScreenW - rPanelW - 14.0f * m_uiScale;
  float rPanelY = contentTop;

  UI::panel("ROAD & TERRAIN METRICS", rPanelX, rPanelY, rPanelW, rPanelH,
            glm::vec4(0.10f, 0.11f, 0.14f, 0.94f));

  float rCurY = rPanelY + 30.0f * m_uiScale;

  // Road Metrics
  UI::label("-- Vector Road Network (OSM / USGS) --",
            rPanelX + 10.0f * m_uiScale, rCurY, 0.82f * m_uiScale,
            glm::vec4(1.0f, 0.75f, 0.3f, 1.0f));
  rCurY += lineH;

  ss.str("");
  if (m_terrainRenderer.isQuadsphere()) {
    uint8_t face = 0;
    double u = 0.0, v = 0.0;
    Quadsphere::geodeticToQuadsphere(m_currentLat, m_currentLon, face, u, v);
    double tilesPerSide = static_cast<double>(1ULL << 15);
    double span = 2.0 / tilesPerSide;
    int curTx = std::clamp(static_cast<int>(std::floor((u + 1.0) / span)), 0, (1 << 15) - 1);
    int curTy = std::clamp(static_cast<int>(std::floor((v + 1.0) / span)), 0, (1 << 15) - 1);

    ss << "Chunk (L15): Face " << static_cast<int>(face) << " [" << curTx << ", " << curTy << "]";
    UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.88f * m_uiScale,
              glm::vec4(0.25f, 0.85f, 1.0f, 1.0f));
    rCurY += lineH;

    int activeQuadChunks = 0;
    size_t streamRoadTris = 0;
    size_t streamRoadVerts = 0;
    size_t streamSegments = 0;
    for (const auto &kv : m_loadedQuadMeshes) {
      if (kv.second.mesh && kv.second.mesh->indexCount > 0) {
        activeQuadChunks++;
        streamRoadTris += (kv.second.mesh->indexCount / 3);
        streamRoadVerts += kv.second.mesh->vertexCount;
        streamSegments += kv.second.segments.size();
      }
    }
    ss.str("");
    ss << "L15 Chunks: " << activeQuadChunks << " loaded (" << streamSegments << " segs)";
    UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.88f * m_uiScale,
              glm::vec4(0.25f, 0.85f, 1.0f, 1.0f));
    rCurY += lineH;

    ss.str("");
    ss << "Road Triangles: " << streamRoadTris << " (" << streamRoadVerts << " verts)";
    UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.88f * m_uiScale,
              glm::vec4(0.35f, 0.95f, 0.55f, 1.0f));
    rCurY += lineH;
  } else {
    int curChunkX = static_cast<int>(std::floor((m_playerPos.x + 256.0f) / 512.0f));
    int curChunkZ = static_cast<int>(std::floor((m_playerPos.z + 256.0f) / 512.0f));
    ss << "Active Chunk: [" << curChunkX << ", " << curChunkZ << "]";
    if (!m_chunksDir.empty()) {
      int activeMeshes = 0;
      for (const auto &kv : m_loadedChunkMeshes) {
        if (kv.second && kv.second->indexCount > 0)
          activeMeshes++;
      }
      ss << " | Loaded: " << activeMeshes << " chunks";
    }
    UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.88f * m_uiScale,
              glm::vec4(0.25f, 0.85f, 1.0f, 1.0f));
    rCurY += lineH;

    ss.str("");
    ss << "Segments: " << m_roadSegments.size() << " | Length: " << std::fixed
       << std::setprecision(2) << (m_totalRoadLengthMeters / 1000.0f) << " km";
    UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.88f * m_uiScale,
              glm::vec4(0.95f, 0.90f, 0.40f, 1.0f));
    rCurY += lineH;

    ss.str("");
    ss << "Junctions: " << m_roadJunctions.size()
       << " | Bridges: " << m_elevatedBridgeCount << " elevated";
    UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.84f * m_uiScale,
              glm::vec4(0.85f, 0.85f, 0.90f, 1.0f));
    rCurY += lineH;

    size_t streamRoadTris = 0;
    size_t streamRoadVerts = 0;
    for (const auto &kv : m_loadedChunkMeshes) {
      if (kv.second) {
        streamRoadTris += (kv.second->indexCount / 3);
        streamRoadVerts += kv.second->vertexCount;
      }
    }
    size_t displayRoadTris =
        (streamRoadTris > 0) ? streamRoadTris : m_totalRoadTriangles;
    size_t displayRoadVerts =
        (streamRoadVerts > 0) ? streamRoadVerts : m_totalRoadVertices;

    ss.str("");
    ss << "Road Triangles: " << displayRoadTris << " ("
       << displayRoadVerts << " verts)";
    UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.88f * m_uiScale,
              glm::vec4(0.35f, 0.95f, 0.55f, 1.0f));
    rCurY += lineH;
  }

  UI::separator(rPanelX + 8.0f * m_uiScale, rCurY, rPanelW - 16.0f * m_uiScale);
  rCurY += 8.0f * m_uiScale;

  // Terrain Metrics
  UI::label("-- Hierarchical Quadtree Terrain --", rPanelX + 10.0f * m_uiScale,
            rCurY, 0.82f * m_uiScale, glm::vec4(0.4f, 0.7f, 1.0f, 1.0f));
  rCurY += lineH;

  const auto &pack = m_terrainRenderer.getPack();
  bool isSomerset =
      (m_terrainRenderer.getPackPath().find("somerset") != std::string::npos);
  std::string dsName = "0.5m LERC Jughandle, NJ";
  if (m_terrainRenderer.isQuadsphere()) {
    dsName = "Quadsphere 0.5m LiDAR Somerset, NJ";
  } else if (pack.header.tileCount > 5000000) {
    dsName = "0.5m LERC New Jersey Statewide";
  } else if (isSomerset) {
    dsName = "0.5m LERC Somerset, NJ";
  }
  ss.str("");
  if (m_terrainRenderer.isQuadsphere()) {
    ss << dsName << " (36.6K tiles)";
  } else {
    ss << dsName << " (" << pack.header.tileCount << " tiles)";
  }
  UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.84f * m_uiScale,
            glm::vec4(0.9f, 0.9f, 0.95f, 1.0f));
  rCurY += lineH;

  size_t triCount = m_terrainRenderer.getRenderedTileCount() *
                    (m_terrainRenderer.getSharedIndexCount() / 3);
  ss.str("");
  ss << "Rendered Tiles: " << m_terrainRenderer.getRenderedTileCount() << " ("
     << std::fixed << std::setprecision(2) << (triCount / 1000000.0f)
     << " M triangles)";
  UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.88f * m_uiScale,
            glm::vec4(0.25f, 1.0f, 0.55f, 1.0f));
  rCurY += lineH;

  float memMB = (m_terrainRenderer.getCachedTileCount() * 755.0f) / 1024.0f;
  ss.str("");
  ss << "Terrain VRAM: " << m_terrainRenderer.getCachedTileCount() << " / "
     << m_terrainRenderer.getMaxCacheTiles() << " (" << std::setprecision(1)
     << memMB << " MB)";
  UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.84f * m_uiScale,
            glm::vec4(0.85f, 0.85f, 0.9f, 1.0f));
  rCurY += lineH;

  // Active LOD breakdown
  UI::separator(rPanelX + 8.0f * m_uiScale, rCurY, rPanelW - 16.0f * m_uiScale);
  rCurY += 6.0f * m_uiScale;

  UI::label("-- Active Terrain LOD Breakdown --", rPanelX + 10.0f * m_uiScale,
            rCurY, 0.80f * m_uiScale, glm::vec4(0.7f, 0.7f, 0.75f, 1.0f));
  rCurY += lineH * 0.9f;

  const auto &lodCounts = m_terrainRenderer.getLodCounts();

  if (m_terrainRenderer.isQuadsphere()) {
    // Quadsphere Level 10-18 (indices 0..8 in lodCounts)
    const char *quadLodNames[9] = {
        "LOD 10 (9.8km, 76m):", "LOD 11 (4.9km, 38m):", "LOD 12 (2.4km, 19m):",
        "LOD 13 (1.2km, 9.5m):", "LOD 14 (610m, 4.8m):", "LOD 15 (305m, 2.4m):",
        "LOD 16 (153m, 1.2m):", "LOD 17 (76m,  0.59m):", "LOD 18 (38m,  0.30m):"
    };
    glm::vec4 quadLodColors[9] = {
        glm::vec4(0.30f, 0.40f, 0.85f, 1.0f),
        glm::vec4(0.40f, 0.50f, 0.95f, 1.0f),
        glm::vec4(0.80f, 0.30f, 0.95f, 1.0f),
        glm::vec4(0.95f, 0.25f, 0.25f, 1.0f),
        glm::vec4(1.00f, 0.55f, 0.15f, 1.0f),
        glm::vec4(0.95f, 0.90f, 0.25f, 1.0f),
        glm::vec4(0.25f, 0.90f, 0.40f, 1.0f),
        glm::vec4(0.20f, 0.85f, 0.95f, 1.0f),
        glm::vec4(0.85f, 0.95f, 1.00f, 1.0f)
    };
    for (int l = 8; l >= 0; --l) {
      uint32_t count = (l < (int)lodCounts.size()) ? lodCounts[l] : 0;
      if (count > 0 || l >= 4) {
        ss.str("");
        ss << quadLodNames[l] << " " << count << " tiles";
        UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.78f * m_uiScale,
                  quadLodColors[l]);
        rCurY += lineH * 0.88f;
      }
    }
  } else {
    const char *lodNames[8] = {
        "LOD 0 (64m,  0.5m):",   "LOD 1 (128m, 1.0m):",  "LOD 2 (256m, 2.0m):",
        "LOD 3 (512m, 4.0m):",   "LOD 4 (1.0km, 8.0m):", "LOD 5 (2.0km, 16.0m):",
        "LOD 6 (4.1km, 32.0m):", "LOD 7 (8.2km, 64.0m):"};
    glm::vec4 lodColors[8] = {glm::vec4(0.20f, 0.85f, 0.95f, 1.0f),
                              glm::vec4(0.25f, 0.90f, 0.40f, 1.0f),
                              glm::vec4(0.95f, 0.90f, 0.25f, 1.0f),
                              glm::vec4(1.00f, 0.55f, 0.15f, 1.0f),
                              glm::vec4(0.95f, 0.25f, 0.25f, 1.0f),
                              glm::vec4(0.80f, 0.30f, 0.95f, 1.0f),
                              glm::vec4(0.40f, 0.50f, 0.95f, 1.0f),
                              glm::vec4(0.30f, 0.40f, 0.85f, 1.0f)};

    for (int l = 0; l <= std::min((int)pack.header.maxLOD, 6); ++l) {
      ss.str("");
      ss << lodNames[l] << " " << (l < (int)lodCounts.size() ? lodCounts[l] : 0)
         << " tiles";
      UI::label(ss.str(), rPanelX + 10.0f * m_uiScale, rCurY, 0.78f * m_uiScale,
                lodColors[l]);
      rCurY += lineH * 0.88f;
    }
  }
}
