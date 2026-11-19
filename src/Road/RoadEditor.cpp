#include "Road/RoadEditor.h"
#include "Road/GeoJsonLoader.h"
#include "Road/RoadGenerator.h"
#include "Road/RoadBinary.h"
#include "Road/RoadElevationSolver.h"
#include "Terrain/TerrainLoader.h"
#include "UI/UIRenderer.h"
#include "Engine.h"
#include "PrimitiveBuilder.h"
#include <glm/glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <chrono>

RoadEditor::RoadEditor() {
}

RoadEditor::~RoadEditor() {
    UI::shutdown();
}

bool RoadEditor::init(Engine& engine) {
    m_engine = &engine;
    SDL_Window* window = engine.getWindow();

    // Ensure mouse cursor is visible and free to interact with UI
    SDL_ShowCursor();
    SDL_SetWindowRelativeMouseMode(window, false);

    // Detect HiDPI scale factor
    int logW = 1280, logH = 800;
    SDL_GetWindowSize(window, &logW, &logH);
    int pixW = 1280, pixH = 800;
    SDL_GetWindowSizeInPixels(window, &pixW, &pixH);

    m_displayScale = SDL_GetWindowDisplayScale(window);
    if (m_displayScale <= 0.0f && logW > 0) {
        m_displayScale = (float)pixW / (float)logW;
    }
    if (m_displayScale <= 0.0f) m_displayScale = 1.0f;

    // Set default UI scale to 1.0
    m_uiScale = 1.0f;

    std::cout << "RoadEditor: Display scale detected = " << m_displayScale
              << " (Logical: " << logW << "x" << logH
              << ", Backbuffer: " << pixW << "x" << pixH
              << ", UI Scale: " << m_uiScale << "x)" << std::endl;

    // 1. Initialize Native Immediate-Mode UI
    if (!UI::init()) {
        std::cerr << "RoadEditor: Failed to initialize UI!" << std::endl;
        return false;
    }

    // 2. Initialize Passable Vehicle Simulator
    m_vehicle.init();

    // 3. Initialize Terrain Pack Container
    initTerrain();

    // 4. Initialize 1m Grid Pipeline & Chunk Terrain Mesh
    initGrid();

    // 5. Initialize Road Pipeline
    initRoadPipeline();

    // 6. Ingest Raw Road Data or Initial Compiled Road Chunk (.froad)
    if (m_isQuadsphere && std::ifstream(m_chunksDir + "/chunk_2_16040_20958.froad").good()) {
        m_activeFace = 2;
        m_activeChunkX = 16040;
        m_activeChunkZ = 20958;
        loadChunk(m_activeFace, m_activeChunkX, m_activeChunkZ);
    } else {
        m_activeChunkX = 0;
        m_activeChunkZ = 0;
        if (!std::ifstream(m_chunksDir + "/chunk_0_0.froad").good() &&
            std::ifstream(m_chunksDir + "/chunk_-1_0.froad").good()) {
            m_activeChunkX = -1;
            m_activeChunkZ = 0;
        }
        loadChunk(m_activeChunkX, m_activeChunkZ);
    }

    if (m_segments.empty() && std::ifstream("assets/maps/jughandle_500m.froad").good()) {
        loadCompiledChunk("assets/maps/jughandle_500m.froad");
    } else if (m_segments.empty() && std::ifstream("jughandle/export.geojson").good()) {
        loadRawGeoJson("jughandle/export.geojson");
    }

    // Focus camera near the chunk center on ground terrain
    if (m_isQuadsphereChunk) {
        m_cameraFocus = glm::vec3(0.0f, 1.0f, 0.0f);
        m_topDownPos = glm::vec3(0.0f, 350.0f, 0.0f);
    } else {
        float centerX = m_activeChunkX * 512.0f;
        float centerZ = m_activeChunkZ * 512.0f;
        float centerY = m_hasTerrain ? sampleTerrainHeight(centerX, centerZ) : 0.0f;
        m_cameraFocus = glm::vec3(centerX, centerY + 1.0f, centerZ);
        m_topDownPos = glm::vec3(centerX, centerY + 350.0f, centerZ);
    }
    m_orbitDistance = 220.0f;
    m_orbitPitch = 48.0f;
    m_orbitYaw = -25.0f;

    std::cout << "RoadEditor: Initialized successfully!" << std::endl;
    return true;
}

void RoadEditor::initTerrain() {
    m_hasTerrain = false;
    m_isQuadsphere = false;

    // Check for Rotated Gnomonic Quadsphere packs first (SPHERE.md)
    std::string quadPath = "jughandle/somerset.fquadspherepack";
    if (!std::ifstream(quadPath).good()) {
        quadPath = "jughandle/new_jersey.fquadspherepack";
    }

    if (std::ifstream(quadPath).good() && QuadsphereLoader::openPack(quadPath, m_quadPack)) {
        m_hasTerrain = true;
        m_isQuadsphere = true;
        std::cout << "RoadEditor: Loaded Quadsphere pack container: " << quadPath
                  << " (" << m_quadPack.header.tileCount << " tiles, LOD "
                  << m_quadPack.header.minLOD << ".." << m_quadPack.header.maxLOD << ")" << std::endl;
        std::cout << "  ECEF Tangent Anchor: (40.592676, -74.515211)" << std::endl;
        return;
    }

    // Fallback to legacy planar terrain pack container
    std::string packPath = "jughandle/new_jersey_05m.fterrainpack";
    if (!std::ifstream(packPath).good()) {
        packPath = "jughandle/somerset_mc_lerc.fterrainpack";
    }

    m_hasTerrain = TerrainLoader::openPack(packPath, m_terrainPack);
    if (m_hasTerrain) {
        std::cout << "RoadEditor: Loaded planar terrain pack container: " << packPath
                  << " (" << m_terrainPack.header.tileCount << " tiles, LOD 0.."
                  << m_terrainPack.header.maxLOD << ")" << std::endl;
        std::cout << "  Geodetic Anchor: (" << m_terrainPack.header.geodeticAnchor[0]
                  << ", " << m_terrainPack.header.geodeticAnchor[1] << ")" << std::endl;
    } else {
        std::cerr << "RoadEditor: Could not open any terrain pack (Quadsphere or planar)!" << std::endl;
    }
}

bool RoadEditor::getTerrainHeight(float localX, float localZ, float& outHeight) {
    if (!m_hasTerrain) return false;

    if (m_isQuadsphere) {
        if (!m_quadPack.isValid() || m_quadPack.indexTable.empty()) return false;

        double lat = 0.0, lon = 0.0;
        if (m_isQuadsphereChunk && glm::length(m_chunkCenterECEF) > 1000.0) {
            double cLat, cLon, cAlt;
            Quadsphere::ecefToGeodetic(m_chunkCenterECEF, cLat, cLon, cAlt);
            glm::dvec3 posENU(localX, 0.0, localZ);
            glm::dvec3 posECEF = m_chunkCenterECEF + glm::dmat3(Quadsphere::getENUtoECEF(cLat, cLon)) * posENU;
            Quadsphere::ecefToGeodetic(posECEF, lat, lon, cAlt);
        } else {
            const double cLat = 40.592676;
            const double cLon = -74.515211;
            const double degToRad = 3.14159265358979323846 / 180.0;
            const double m_per_lat = 111139.0;
            const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

            lat = cLat - (static_cast<double>(localZ) / m_per_lat);
            lon = cLon + (static_cast<double>(localX) / m_per_lon);
        }

        uint8_t face; double u, v;
        Quadsphere::geodeticToQuadsphere(lat, lon, face, u, v);

        // Search pyramid from highest resolution (maxLOD) to lowest (minLOD)
        for (int lod = m_quadPack.header.maxLOD; lod >= m_quadPack.header.minLOD; --lod) {
            double tilesPerSide = static_cast<double>(1ULL << lod);
            double span = 2.0 / tilesPerSide;

            int tx = static_cast<int>(std::floor((u + 1.0) / span));
            int ty = static_cast<int>(std::floor((v + 1.0) / span));
            if (tx < 0 || ty < 0 || tx >= (int)tilesPerSide || ty >= (int)tilesPerSide) continue;

            const auto* entry = QuadsphereLoader::findTileIndex(m_quadPack, face, lod, tx, ty);
            if (!entry) continue;

            uint64_t key = entry->key;
            auto it = m_quadTileCache.find(key);
            if (it == m_quadTileCache.end()) {
                DecodedQuadsphereTile tile;
                if (QuadsphereLoader::loadTile(m_quadPack, face, lod, tx, ty, tile)) {
                    it = m_quadTileCache.emplace(key, std::move(tile)).first;
                } else {
                    continue;
                }
            }

            const DecodedQuadsphereTile& tile = it->second;
            double minU, maxU, minV, maxV;
            Quadsphere::getTileUVBounds(lod, tx, ty, minU, maxU, minV, maxV);

            double fracX = std::clamp((u - minU) / (maxU - minU), 0.0, 1.0);
            double fracY = std::clamp((v - minV) / (maxV - minV), 0.0, 1.0);

            double fx = fracX * 128.0;
            double fy = fracY * 128.0;
            int ix = std::clamp(static_cast<int>(fx), 0, 127);
            int iy = std::clamp(static_cast<int>(fy), 0, 127);
            float rx = static_cast<float>(fx - ix);
            float ry = static_cast<float>(fy - iy);

            float h00 = tile.getElevation(ix, iy);
            float h10 = tile.getElevation(ix + 1, iy);
            float h01 = tile.getElevation(ix, iy + 1);
            float h11 = tile.getElevation(ix + 1, iy + 1);

            float h0 = h00 * (1.0f - rx) + h10 * rx;
            float h1 = h01 * (1.0f - rx) + h11 * rx;
            outHeight = h0 * (1.0f - ry) + h1 * ry;
            return true;
        }

        return false;
    }

    float relE = localX - m_terrainPack.startEast;
    float relN = -localZ - m_terrainPack.startNorth;

    float maxE = m_terrainPack.tilesX_LOD0 * m_terrainPack.header.baseTileSize;
    float maxN = m_terrainPack.tilesY_LOD0 * m_terrainPack.header.baseTileSize;

    if (relE < 0.0f || relN < 0.0f || relE > maxE || relN > maxN) {
        return false;
    }

    // Search pyramid from highest resolution (LOD 0) to lowest
    for (uint16_t lod = 0; lod <= m_terrainPack.header.maxLOD; ++lod) {
        float tileSz = m_terrainPack.header.baseTileSize * static_cast<float>(1 << lod);
        if (relE < 0.0f || relN < 0.0f) continue;
        uint32_t tx = static_cast<uint32_t>(relE / tileSz);
        uint32_t ty = static_cast<uint32_t>(relN / tileSz);
        uint64_t key = TerrainLoader::packTileKey(lod, tx, ty);

        const auto* entry = TerrainLoader::findTileIndex(m_terrainPack, lod, tx, ty);
        if (entry) {
            // Skip flat border padding tiles outside the map
            if (entry->maxElevation - entry->minElevation < 0.001f) {
                return false;
            }

            auto it = m_tileCache.find(key);
            if (it != m_tileCache.end()) {
                float localTileX = relE - tx * tileSz;
                float localTileZ = relN - ty * tileSz;
                outHeight = TerrainLoader::sampleTileHeight(it->second, localTileX, localTileZ, tileSz);
                return true;
            }

            DecodedTerrainTile tile;
            if (TerrainLoader::loadTile(m_terrainPack, lod, tx, ty, tile)) {
                float localTileX = relE - tx * tileSz;
                float localTileZ = relN - ty * tileSz;
                outHeight = TerrainLoader::sampleTileHeight(tile, localTileX, localTileZ, tileSz);
                m_tileCache[key] = std::move(tile);
                return true;
            }
        }
    }

    return false;
}

float RoadEditor::sampleTerrainHeight(float localX, float localZ) {
    float h = 0.0f;
    if (getTerrainHeight(localX, localZ, h)) {
        return h;
    }
    return 0.0f;
}

void RoadEditor::buildChunkTerrainMesh() {
    m_terrainMesh = nullptr;
    m_gridMesh = nullptr;
    if (!m_hasTerrain) return;

    if (m_isQuadsphereChunk && m_isQuadsphere) {
        double minU, maxU, minV, maxV;
        Quadsphere::getTileUVBounds(m_activeLOD, m_activeChunkX, m_activeChunkZ, minU, maxU, minV, maxV);
        double uCenter = (minU + maxU) * 0.5;
        double vCenter = (minV + maxV) * 0.5;
        glm::dvec3 centerECEF = Quadsphere::quadsphereToECEF(m_activeFace, uCenter, vCenter, 0.0);
        m_chunkCenterECEF = centerECEF;

        double cLat, cLon, cAlt;
        Quadsphere::ecefToGeodetic(centerECEF, cLat, cLon, cAlt);
        m_R_ecef_to_enu = glm::mat4(Quadsphere::getECEFtoENU(cLat, cLon));
        m_R_enu_to_ecef = glm::mat4(Quadsphere::getENUtoECEF(cLat, cLon));

        const int N = 65; // 65x65 grid of vertices over the tile
        std::vector<Vertex> vertices(N * N);
        std::vector<bool> valid(N * N, false);
        int validCount = 0;

        for (int iz = 0; iz < N; ++iz) {
            double vFrac = static_cast<double>(iz) / static_cast<double>(N - 1);
            double v = minV + vFrac * (maxV - minV);
            for (int ix = 0; ix < N; ++ix) {
                double uFrac = static_cast<double>(ix) / static_cast<double>(N - 1);
                double u = minU + uFrac * (maxU - minU);
                int idx = iz * N + ix;

                glm::dvec3 n_cube = Quadsphere::getCubeFaceVector(m_activeFace, u, v);
                glm::dvec3 d_cube = glm::normalize(n_cube);
                static const glm::dmat3 R_cube = Quadsphere::getCubeRotationMatrix();
                glm::dvec3 n_up = R_cube * d_cube;

                double ptLat, ptLon, ptH;
                Quadsphere::ecefToGeodetic(n_up * Quadsphere::EARTH_RADIUS, ptLat, ptLon, ptH);

                float elev = 0.0f;
                if (QuadsphereLoader::hasElevation(m_quadPack, ptLat, ptLon, elev)) {
                    valid[idx] = true;
                    validCount++;
                }

                glm::dvec3 ptECEF = (Quadsphere::EARTH_RADIUS + elev) * n_up;
                glm::vec3 relECEF = glm::vec3(ptECEF - centerECEF);
                glm::vec3 posENU = glm::vec3(m_R_ecef_to_enu * glm::vec4(relECEF, 0.0f));
                glm::vec3 normENU = glm::normalize(glm::vec3(m_R_ecef_to_enu * glm::vec4(glm::vec3(n_up), 0.0f)));

                Vertex& vert = vertices[idx];
                vert.position[0] = posENU.x;
                vert.position[1] = posENU.y;
                vert.position[2] = posENU.z;
                vert.normal[0] = normENU.x;
                vert.normal[1] = normENU.y;
                vert.normal[2] = normENU.z;
                vert.texCoords[0] = posENU.x;
                vert.texCoords[1] = posENU.z;
                vert.tangent[0] = 1.0f;
                vert.tangent[1] = 0.0f;
                vert.tangent[2] = 0.0f;
            }
        }

        if (validCount > 0) {
            std::vector<uint32_t> indices;
            indices.reserve((N - 1) * (N - 1) * 6);
            for (int iz = 0; iz < N - 1; ++iz) {
                for (int ix = 0; ix < N - 1; ++ix) {
                    uint32_t i0 = static_cast<uint32_t>(iz * N + ix);
                    uint32_t i1 = i0 + 1;
                    uint32_t i2 = static_cast<uint32_t>((iz + 1) * N + ix);
                    uint32_t i3 = i2 + 1;

                    indices.push_back(i0);
                    indices.push_back(i2);
                    indices.push_back(i1);

                    indices.push_back(i1);
                    indices.push_back(i2);
                    indices.push_back(i3);
                }
            }

            m_terrainMesh = std::make_shared<Mesh>(vertices.data(), static_cast<Uint32>(vertices.size()),
                                                   indices.data(), static_cast<Uint32>(indices.size()));
            if (m_showTerrain) {
                m_gridMesh = m_terrainMesh;
            }
        }
        return;
    }

    // Road chunk footprint: 512m x 512m centered at (m_activeChunkX * 512.0f, m_activeChunkZ * 512.0f).
    // Terrain is rendered strictly for this active 512x512m chunk
    float centerX = m_activeChunkX * 512.0f;
    float centerZ = m_activeChunkZ * 512.0f;
    const float minX = centerX - 256.0f;
    const float maxX = centerX + 256.0f;
    const float minZ = centerZ - 256.0f;
    const float maxZ = centerZ + 256.0f;
    const float step = 2.0f;

    const int countX = static_cast<int>((maxX - minX) / step) + 1; // 257
    const int countZ = static_cast<int>((maxZ - minZ) / step) + 1; // 257

    std::vector<float> heights(countX * countZ, 0.0f);
    std::vector<bool> valid(countX * countZ, false);
    int validCount = 0;

    for (int iz = 0; iz < countZ; ++iz) {
        float z = minZ + static_cast<float>(iz) * step;
        for (int ix = 0; ix < countX; ++ix) {
            float x = minX + static_cast<float>(ix) * step;
            float h = 0.0f;
            int idx = iz * countX + ix;
            if (getTerrainHeight(x, z, h)) {
                heights[idx] = h;
                valid[idx] = true;
                validCount++;
            }
        }
    }

    // If active chunk has no valid terrain, stop cleanly (no flat fallback terrain!)
    if (validCount == 0) {
        return;
    }

    std::vector<Vertex> vertices(countX * countZ);

    for (int iz = 0; iz < countZ; ++iz) {
        float z = minZ + static_cast<float>(iz) * step;
        for (int ix = 0; ix < countX; ++ix) {
            int idx = iz * countX + ix;
            float x = minX + static_cast<float>(ix) * step;
            float y = heights[idx];

            float hL = (ix > 0 && valid[idx - 1]) ? heights[idx - 1] : y;
            float hR = (ix < countX - 1 && valid[idx + 1]) ? heights[idx + 1] : y;
            float hD = (iz > 0 && valid[idx - countX]) ? heights[idx - countX] : y;
            float hU = (iz < countZ - 1 && valid[idx + countX]) ? heights[idx + countX] : y;

            glm::vec3 normal = glm::normalize(glm::vec3(hL - hR, 2.0f * step, hD - hU));
            glm::vec3 tangent = glm::vec3(1.0f, 0.0f, 0.0f);
            glm::vec2 uv = glm::vec2(x, z);

            Vertex& v = vertices[idx];
            v.position[0] = x;
            v.position[1] = y;
            v.position[2] = z;
            v.normal[0] = normal.x;
            v.normal[1] = normal.y;
            v.normal[2] = normal.z;
            v.texCoords[0] = uv.x;
            v.texCoords[1] = uv.y;
            v.tangent[0] = tangent.x;
            v.tangent[1] = tangent.y;
            v.tangent[2] = tangent.z;
        }
    }

    std::vector<uint32_t> indices;
    indices.reserve((countX - 1) * (countZ - 1) * 6);

    for (int iz = 0; iz < countZ - 1; ++iz) {
        for (int ix = 0; ix < countX - 1; ++ix) {
            uint32_t i0 = static_cast<uint32_t>(iz * countX + ix);
            uint32_t i1 = i0 + 1;
            uint32_t i2 = static_cast<uint32_t>((iz + 1) * countX + ix);
            uint32_t i3 = i2 + 1;

            // Only generate triangles for this quad if all four corners have valid terrain
            if (valid[i0] && valid[i1] && valid[i2] && valid[i3]) {
                indices.push_back(i0);
                indices.push_back(i2);
                indices.push_back(i1);

                indices.push_back(i1);
                indices.push_back(i2);
                indices.push_back(i3);
            }
        }
    }

    if (!indices.empty()) {
        m_terrainMesh = std::make_shared<Mesh>(vertices.data(), static_cast<Uint32>(vertices.size()),
                                               indices.data(), static_cast<Uint32>(indices.size()));
        if (m_showTerrain) {
            m_gridMesh = m_terrainMesh;
        }
        std::cout << "RoadEditor: Built chunk terrain mesh: " << vertices.size() << " vertices, "
                  << indices.size() / 3 << " triangles covering [" << minX << "m, " << maxX << "m] x ["
                  << minZ << "m, " << maxZ << "m]" << std::endl;
    }
}

void RoadEditor::solveRoadElevations() {
    if (m_rawFeatures.empty() && m_segments.empty()) return;

    // Only populate from raw features if we have no existing segments.
    // Preserves all manual edits (dragged control points, custom lanes, widths, splits, layers).
    if (m_segments.empty()) {
        RoadGenerator::processFeatures(m_rawFeatures, m_laneWidth, m_shoulderWidth, m_sampleStep, m_segments, false);
    }

    if (m_hasTerrain) {
        auto startT = std::chrono::high_resolution_clock::now();

        if (m_isQuadsphereChunk && m_isQuadsphere) {
            auto sampleTerrainQuad = [this](double latDeg, double lonDeg) -> float {
                float elev = 0.0f;
                QuadsphereLoader::hasElevation(m_quadPack, latDeg, lonDeg, elev);
                return elev;
            };
            RoadElevationSolver::solveQuadsphere(m_segments, sampleTerrainQuad, m_solverOptions);
        } else {
            auto sampleFn = [this](float x, float z) -> float {
                return sampleTerrainHeight(x, z);
            };
            RoadElevationSolver::solve(m_segments, sampleFn, m_solverOptions);
        }

        auto endT = std::chrono::high_resolution_clock::now();
        m_lastSolveTimeMs = std::chrono::duration<float, std::milli>(endT - startT).count();
        m_lastSolveSweeps = m_solverOptions.maxSweeps;
    }

    rebuildTopologyAndGeometry();

    std::cout << "RoadEditor: Solved road elevations: " << m_segments.size()
              << " segments, " << m_junctions.size() << " junctions. Merged mesh: "
              << (m_mergedMesh ? m_mergedMesh->vertexCount : 0) << " vertices, "
              << (m_mergedMesh ? (m_mergedMesh->indexCount / 3) : 0) << " triangles." << std::endl;
}

void RoadEditor::initGrid() {
    auto& device = RHI_Device::getInstance();

    m_gridShader = std::make_shared<Shader>("assets/shaders/grid.hlsl", "assets/shaders/grid.hlsl");

    m_flatGridMesh = nullptr;

    if (m_hasTerrain) {
        buildChunkTerrainMesh();
        m_gridMesh = m_terrainMesh;
    } else {
        m_gridMesh = nullptr;
    }

    SDL_GPUVertexAttribute attributes[4] = {};
    attributes[0].location = 0;
    attributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attributes[0].offset = offsetof(Vertex, position);

    attributes[1].location = 1;
    attributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attributes[1].offset = offsetof(Vertex, normal);

    attributes[2].location = 2;
    attributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
    attributes[2].offset = offsetof(Vertex, texCoords);

    attributes[3].location = 3;
    attributes[3].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
    attributes[3].offset = offsetof(Vertex, tangent);

    SDL_GPUVertexBufferDescription binding[1] = {};
    binding[0].slot = 0;
    binding[0].pitch = sizeof(Vertex);
    binding[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

    m_gridPipeline = std::make_shared<RHI_Pipeline>();
    m_gridPipeline->create(m_gridShader->vertexShader.get(), m_gridShader->fragmentShader.get(),
                           attributes, 4,
                           binding, 1,
                           device.getSwapchainFormat(),
                           SDL_GPU_TEXTUREFORMAT_D32_FLOAT,
                           true /* depthTest */);
}

void RoadEditor::initRoadPipeline() {
    auto& device = RHI_Device::getInstance();
    m_roadRenderer.init(device.getSwapchainFormat(), SDL_GPU_TEXTUREFORMAT_D32_FLOAT);
}

void RoadEditor::loadRawGeoJson(const std::string& path) {
    double centerLon = -74.515211, centerLat = 40.592676;
    if (GeoJsonLoader::load(path, m_rawFeatures, centerLon, centerLat)) {
        rebuildRoadGeometry();
        m_statusMessage = "Loaded " + std::to_string(m_rawFeatures.size()) + " features from " + path;
        m_statusTimer = 4.0f;
    } else {
        m_statusMessage = "Error loading " + path;
        m_statusTimer = 4.0f;
    }
}

void RoadEditor::rebuildRoadGeometry() {
    if (m_hasTerrain && m_autoSolveElevation) {
        solveRoadElevations();
    } else {
        RoadGenerator::processFeatures(m_rawFeatures, m_laneWidth, m_shoulderWidth, m_sampleStep, m_segments, false);
        rebuildTopologyAndGeometry();
    }
    m_selectedSegmentIdx = -1;
    m_selectedMesh = nullptr;
    m_selectedControlPointIdx = -1;
    m_draggedControlPointIdx = -1;
    m_isDraggingControlPoint = false;
}

void RoadEditor::rebuildTopologyAndGeometry() {
    float halfS = 256.0f;
    float minX = m_activeChunkX * 512.0f - halfS;
    float maxX = m_activeChunkX * 512.0f + halfS;
    float minZ = m_activeChunkZ * 512.0f - halfS;
    float maxZ = m_activeChunkZ * 512.0f + halfS;

    // 0. Automatically collapse short roads squeezed between two junctions
    if (RoadGenerator::collapseShortJunctionSegments(m_segments, 26.0f)) {
        if (m_selectedSegmentIdx >= (int)m_segments.size()) {
            m_selectedSegmentIdx = (int)m_segments.size() - 1;
        }
    }

    // 1. Re-generate splines for all segments from their control points
    for (auto& seg : m_segments) {
        seg.totalWidth = (float)seg.lanes * seg.laneWidth + 2.0f * seg.shoulderWidth;
        if (m_isQuadsphereChunk) {
            seg.splinePoints = RoadGenerator::generateSpline(seg.controlPoints, seg.sampleStep, &m_chunkCenterECEF);
            seg.in500mChunk = true;
        } else {
            seg.splinePoints = RoadGenerator::generateSpline(seg.controlPoints, seg.sampleStep);
            if (seg.splinePoints.size() >= 2) {
                bool inChunk = false;
                for (const auto& sp : seg.splinePoints) {
                    if (sp.pos.x >= minX && sp.pos.x <= maxX && sp.pos.z >= minZ && sp.pos.z <= maxZ) {
                        inChunk = true;
                        break;
                    }
                }
                seg.in500mChunk = inChunk;
            }
        }
        if (!seg.splinePoints.empty()) {
            seg.length = seg.splinePoints.back().distance;
        }
    }

    // 2. Re-detect network topology and junctions
    RoadGenerator::buildNetworkTopology(m_segments, m_junctions);

    // 3. Re-generate junction patch plates, fillets, and setbacks
    RoadGenerator::generateJunctionPatches(m_segments, m_junctions, m_isQuadsphereChunk ? &m_chunkCenterECEF : nullptr);

    // 4. Rebuild individual segment visual meshes
    for (auto& seg : m_segments) {
        auto trimmed = RoadGenerator::trimSpline(seg.splinePoints, seg.startSetback, seg.endSetback);
        std::vector<Vertex> verts;
        std::vector<uint32_t> idxs;
        RoadGenerator::generateRibbonGeometry(trimmed, seg.totalWidth, seg.lanes, seg.oneway, seg.shoulderWidth, verts, idxs);
        RoadGenerator::clipRibbonAgainstJunctions(verts, seg, m_junctions, m_segments);
        if (!verts.empty() && !idxs.empty()) {
            seg.visualMesh = std::make_shared<Mesh>(verts.data(), (Uint32)verts.size(), idxs.data(), (Uint32)idxs.size());
        }
    }

    // 5. Rebuild merged mesh (filtered to active chunk bounds when filter enabled)
    m_mergedMesh = RoadGenerator::buildMergedMesh(m_segments, m_junctions, m_isQuadsphereChunk ? false : m_filter500mOnly, minX, maxX, minZ, maxZ);

    if (m_selectedSegmentIdx >= 0 && m_selectedSegmentIdx < (int)m_segments.size()) {
        m_selectedMesh = m_segments[m_selectedSegmentIdx].visualMesh;
    } else {
        m_selectedMesh = nullptr;
    }
}

void RoadEditor::splitSegmentAtControlPoint(int segIdx, int cpIdx) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    auto& orig = m_segments[segIdx];
    if (cpIdx <= 0 || cpIdx >= (int)orig.controlPoints.size() - 1) return;

    // Create new segment taking control points from cpIdx to the end
    ProcessedRoadSegment newSeg;
    uint32_t maxId = 0;
    for (const auto& s : m_segments) maxId = std::max(maxId, s.id);
    newSeg.id = maxId + 1;
    newSeg.osmId = orig.osmId + "_split";
    newSeg.name = orig.name + " (Part 2)";
    newSeg.highway = orig.highway;
    newSeg.ref = orig.ref;
    newSeg.lanes = orig.lanes;
    newSeg.oneway = orig.oneway;
    newSeg.laneWidth = orig.laneWidth;
    newSeg.shoulderWidth = orig.shoulderWidth;
    newSeg.totalWidth = orig.totalWidth;
    newSeg.sampleStep = orig.sampleStep;
    newSeg.widthSource = orig.widthSource;

    // Split control points:
    // orig retains [0 .. cpIdx]
    // newSeg gets [cpIdx .. end]
    newSeg.controlPoints.assign(orig.controlPoints.begin() + cpIdx, orig.controlPoints.end());
    orig.controlPoints.erase(orig.controlPoints.begin() + cpIdx + 1, orig.controlPoints.end());

    m_segments.push_back(newSeg);

    // Rebuild topology and geometry
    rebuildTopologyAndGeometry();

    m_selectedSegmentIdx = segIdx;
    m_selectedControlPointIdx = -1;
    if (m_selectedSegmentIdx < (int)m_segments.size()) {
        m_selectedMesh = m_segments[m_selectedSegmentIdx].visualMesh;
    }
    m_statusMessage = "Split road into 2 segments at CP #" + std::to_string(cpIdx) + " (created 2-way junction)";
    m_statusTimer = 4.0f;
}

void RoadEditor::handleJunctionSelectionClick(float mx, float my, int screenW, int screenH) {
    // Avoid clicking on top bar or side panels
    if (my < 45.0f * m_uiScale) return;
    if (mx < 310.0f * m_uiScale) return;
    if (mx > (float)screenW - 350.0f * m_uiScale) return;

    // Avoid clicking on the floating Junction Builder panel
    float jbW = 380.0f * m_uiScale;
    float jbX = ((float)screenW - jbW) * 0.5f;
    float jbY = 50.0f * m_uiScale;
    float jbH = 75.0f * m_uiScale + (float)std::max(1, (int)m_pendingJunctionConns.size()) * (26.0f * m_uiScale) + 46.0f * m_uiScale;
    if (mx >= jbX && mx <= jbX + jbW && my >= jbY && my <= jbY + jbH) return;

    glm::mat4 view = getViewMatrix();
    glm::mat4 proj = getProjMatrix((float)screenW / (float)screenH);
    glm::mat4 vp = proj * view;
    glm::mat4 invVP = glm::inverse(vp);

    float ndcX = (mx / (float)screenW) * 2.0f - 1.0f;
    float ndcY = 1.0f - (my / (float)screenH) * 2.0f;

    glm::vec4 nearPt = invVP * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
    glm::vec4 farPt  = invVP * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
    nearPt /= nearPt.w;
    farPt /= farPt.w;

    glm::vec3 rayOrigin = glm::vec3(nearPt);
    glm::vec3 rayDir = glm::normalize(glm::vec3(farPt - nearPt));

    int bestSegIdx = -1;
    bool bestIsStart = false;
    float bestEndpointScreenDist = 28.0f * m_uiScale;

    // Pass 1: Direct 2D screen-space click on endpoint handles
    for (size_t i = 0; i < m_segments.size(); ++i) {
        const auto& seg = m_segments[i];
        if (seg.controlPoints.size() < 2) continue;
        if (!m_isQuadsphereChunk && m_filter500mOnly && !seg.in500mChunk) continue;

        glm::vec3 pStart = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(seg.controlPoints.front(), 1.0f)) : seg.controlPoints.front();
        glm::vec3 pEnd   = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(seg.controlPoints.back(), 1.0f)) : seg.controlPoints.back();

        // Check start endpoint on screen
        glm::vec4 cStart = vp * glm::vec4(pStart, 1.0f);
        if (cStart.w > 0.01f) {
            glm::vec3 ndc = glm::vec3(cStart) / cStart.w;
            if (ndc.z >= 0.0f && ndc.z <= 1.0f) {
                float sx = (ndc.x * 0.5f + 0.5f) * (float)screenW;
                float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)screenH;
                float dist = glm::distance(glm::vec2(mx, my), glm::vec2(sx, sy));
                if (dist < bestEndpointScreenDist) {
                    bestEndpointScreenDist = dist;
                    bestSegIdx = (int)i;
                    bestIsStart = true;
                }
            }
        }

        // Check end endpoint on screen
        glm::vec4 cEnd = vp * glm::vec4(pEnd, 1.0f);
        if (cEnd.w > 0.01f) {
            glm::vec3 ndc = glm::vec3(cEnd) / cEnd.w;
            if (ndc.z >= 0.0f && ndc.z <= 1.0f) {
                float sx = (ndc.x * 0.5f + 0.5f) * (float)screenW;
                float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)screenH;
                float dist = glm::distance(glm::vec2(mx, my), glm::vec2(sx, sy));
                if (dist < bestEndpointScreenDist) {
                    bestEndpointScreenDist = dist;
                    bestSegIdx = (int)i;
                    bestIsStart = false;
                }
            }
        }
    }

    // Pass 2: If no direct endpoint marker was clicked, 3D raycast against road segment spline points
    if (bestSegIdx < 0) {
        float bestT = 1e9f;
        glm::vec3 bestHitPos(0.0f);

        for (size_t i = 0; i < m_segments.size(); ++i) {
            const auto& seg = m_segments[i];
            if (seg.controlPoints.size() < 2) continue;
            if (!m_isQuadsphereChunk && m_filter500mOnly && !seg.in500mChunk) continue;

            float maxD = std::max(6.0f, seg.totalWidth * 1.5f);
            for (const auto& sp : seg.splinePoints) {
                glm::vec3 spPos = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(sp.pos, 1.0f)) : sp.pos;
                glm::vec3 diff = spPos - rayOrigin;
                float t = glm::dot(diff, rayDir);
                if (t > 0.0f && t < bestT) {
                    glm::vec3 ptOnRay = rayOrigin + rayDir * t;
                    float d3d = glm::distance(ptOnRay, spPos);
                    if (d3d <= maxD) {
                        bestT = t;
                        bestSegIdx = (int)i;
                        bestHitPos = spPos;
                    }
                }
            }
        }

        if (bestSegIdx >= 0) {
            const auto& seg = m_segments[bestSegIdx];
            glm::vec3 pStart = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(seg.controlPoints.front(), 1.0f)) : seg.controlPoints.front();
            glm::vec3 pEnd   = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(seg.controlPoints.back(), 1.0f)) : seg.controlPoints.back();
            float dStart = glm::distance(bestHitPos, pStart);
            float dEnd   = glm::distance(bestHitPos, pEnd);
            bestIsStart = (dStart < dEnd);
        }
    }

    if (bestSegIdx >= 0) {
        const auto& seg = m_segments[bestSegIdx];
        bool isStart = bestIsStart;

        // Toggle selection if already added
        auto it = std::find_if(m_pendingJunctionConns.begin(), m_pendingJunctionConns.end(),
                               [&](const PendingJunctionConnection& c) {
                                   return c.segmentIdx == bestSegIdx && c.isStart == isStart;
                               });
        if (it != m_pendingJunctionConns.end()) {
            m_pendingJunctionConns.erase(it);
            m_statusMessage = "Deselected " + seg.name + (isStart ? " (Start)" : " (End)");
        } else {
            m_pendingJunctionConns.push_back({ bestSegIdx, isStart });
            m_statusMessage = "Selected " + seg.name + (isStart ? " (Start)" : " (End)") +
                              " [" + std::to_string(m_pendingJunctionConns.size()) + " road ends selected]";
        }
        m_statusTimer = 4.0f;
    }
}

void RoadEditor::createJunctionFromPending() {
    if (m_pendingJunctionConns.size() < 2) {
        m_statusMessage = "Select at least 2 road ends to create a junction.";
        m_statusTimer = 3.5f;
        return;
    }

    // 1. Calculate centroid of all selected endpoints
    glm::vec3 centroid(0.0f);
    int validCount = 0;
    for (const auto& conn : m_pendingJunctionConns) {
        if (conn.segmentIdx < 0 || conn.segmentIdx >= (int)m_segments.size()) continue;
        const auto& seg = m_segments[conn.segmentIdx];
        if (seg.controlPoints.empty()) continue;
        glm::vec3 pt = conn.isStart ? seg.controlPoints.front() : seg.controlPoints.back();
        centroid += pt;
        validCount++;
    }
    if (validCount < 2) return;
    centroid /= (float)validCount;
    if (!m_isQuadsphereChunk && !m_hasTerrain) {
        centroid.y = 0.05f;
    }

    // 2. Snap all selected endpoints to the centroid position
    for (const auto& conn : m_pendingJunctionConns) {
        if (conn.segmentIdx < 0 || conn.segmentIdx >= (int)m_segments.size()) continue;
        auto& seg = m_segments[conn.segmentIdx];
        if (seg.controlPoints.empty()) continue;
        if (conn.isStart) {
            seg.controlPoints.front() = centroid;
        } else {
            seg.controlPoints.back() = centroid;
        }
    }

    size_t count = m_pendingJunctionConns.size();
    m_pendingJunctionConns.clear();
    m_isAddingJunction = false;

    // 3. Rebuild topology and all geometry
    rebuildTopologyAndGeometry();

    // Select the newly created junction
    int bestJunc = -1;
    float bestDist2 = 1e9f;
    for (size_t j = 0; j < m_junctions.size(); ++j) {
        glm::vec3 diff = m_junctions[j].position - centroid;
        float d2 = glm::dot(diff, diff);
        if (d2 < bestDist2) {
            bestDist2 = d2;
            bestJunc = (int)j;
        }
    }
    if (bestJunc >= 0 && bestDist2 < 25.0f) {
        selectJunction(bestJunc);
    }

    m_statusMessage = "Successfully created new " + std::to_string(count) + "-way junction!";
    m_statusTimer = 5.0f;
}

void RoadEditor::removeJunction(int juncId) {
    if (juncId < 0 || juncId >= (int)m_junctions.size()) return;
    const auto& junc = m_junctions[juncId];

    // For each connected segment, pull the endpoint back along its tangent so it won't re-merge
    for (const auto& conn : junc.connections) {
        if (conn.segmentIdx >= 0 && conn.segmentIdx < (int)m_segments.size()) {
            auto& seg = m_segments[conn.segmentIdx];
            if (conn.isStart && seg.controlPoints.size() >= 2) {
                glm::vec3 dir = glm::normalize(seg.controlPoints[1] - seg.controlPoints[0]);
                seg.controlPoints.front() += dir * 6.0f;
                seg.startJunctionId = -1;
            } else if (!conn.isStart && seg.controlPoints.size() >= 2) {
                size_t n = seg.controlPoints.size();
                glm::vec3 dir = glm::normalize(seg.controlPoints[n - 2] - seg.controlPoints[n - 1]);
                seg.controlPoints.back() += dir * 6.0f;
                seg.endJunctionId = -1;
            }
        }
    }

    m_selectedJunctionIdx = -1;
    m_selectedMesh = nullptr;
    rebuildTopologyAndGeometry();

    m_statusMessage = "Removed junction #" + std::to_string(juncId);
    m_statusTimer = 3.5f;
}

void RoadEditor::deleteSegment(int segIdx) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    std::string segName = m_segments[segIdx].name;
    int segId = m_segments[segIdx].id;

    m_segments.erase(m_segments.begin() + segIdx);
    m_selectedSegmentIdx = -1;
    m_selectedControlPointIdx = -1;
    m_draggedControlPointIdx = -1;
    m_isDraggingControlPoint = false;
    m_selectedMesh = nullptr;

    rebuildTopologyAndGeometry();

    m_statusMessage = "Deleted road segment #" + std::to_string(segId) + (segName.empty() ? "" : " ('" + segName + "')");
    m_statusTimer = 4.0f;
}

void RoadEditor::disconnectSegmentFromJunction(int segIdx, bool isStart) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    auto& seg = m_segments[segIdx];

    if (isStart) {
        if (seg.controlPoints.size() >= 2) {
            glm::vec3 dir = glm::normalize(seg.controlPoints[1] - seg.controlPoints[0]);
            seg.controlPoints.front() += dir * 4.0f;
        }
        seg.startJunctionId = -1;
    } else {
        if (seg.controlPoints.size() >= 2) {
            size_t n = seg.controlPoints.size();
            glm::vec3 dir = glm::normalize(seg.controlPoints[n - 2] - seg.controlPoints[n - 1]);
            seg.controlPoints.back() += dir * 4.0f;
        }
        seg.endJunctionId = -1;
    }

    rebuildTopologyAndGeometry();
    m_statusMessage = "Disconnected " + seg.name + (isStart ? " [Start]" : " [End]") + " from junction.";
    m_statusTimer = 3.5f;
}

void RoadEditor::recenterOrbitOnTerrain() {
    float radYaw = glm::radians(m_orbitYaw);
    float radPitch = glm::radians(m_orbitPitch);
    glm::vec3 camOffset(
        m_orbitDistance * std::cos(radPitch) * std::sin(radYaw),
        m_orbitDistance * std::sin(radPitch),
        m_orbitDistance * std::cos(radPitch) * std::cos(radYaw)
    );
    glm::vec3 camPos = m_cameraFocus + camOffset;
    glm::vec3 rayDir = -glm::normalize(camOffset);

    // If ray points upwards or nearly parallel to ground, ignore
    if (rayDir.y >= -0.01f) return;

    float maxDist = std::max(2500.0f, m_orbitDistance * 4.0f);
    float step = 2.0f;
    float prevT = 0.0f;
    float tHit = -1.0f;

    for (float t = 1.0f; t < maxDist; t += step) {
        glm::vec3 p = camPos + rayDir * t;
        float groundY = 0.0f;
        if (m_hasTerrain) {
            groundY = sampleTerrainHeight(p.x, p.z);
        }
        if (p.y <= groundY) {
            // Binary search refinement
            float tLow = prevT;
            float tHigh = t;
            for (int iter = 0; iter < 8; ++iter) {
                float tMid = (tLow + tHigh) * 0.5f;
                glm::vec3 pMid = camPos + rayDir * tMid;
                float gy = m_hasTerrain ? sampleTerrainHeight(pMid.x, pMid.z) : 0.0f;
                if (pMid.y <= gy) {
                    tHigh = tMid;
                } else {
                    tLow = tMid;
                }
            }
            tHit = (tLow + tHigh) * 0.5f;
            break;
        }
        prevT = t;
    }

    if (tHit > 0.0f) {
        glm::vec3 newFocus = camPos + rayDir * tHit;
        m_cameraFocus = newFocus;
        m_orbitDistance = std::clamp(tHit, 10.0f, 1500.0f);
    }
}

void RoadEditor::rebuildSegmentGeometry(int segIdx) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    auto& seg = m_segments[segIdx];

    float halfS = 256.0f;
    float minX = m_activeChunkX * 512.0f - halfS;
    float maxX = m_activeChunkX * 512.0f + halfS;
    float minZ = m_activeChunkZ * 512.0f - halfS;
    float maxZ = m_activeChunkZ * 512.0f + halfS;

    seg.totalWidth = (float)seg.lanes * seg.laneWidth + 2.0f * seg.shoulderWidth;
    if (m_isQuadsphereChunk) {
        seg.splinePoints = RoadGenerator::generateSpline(seg.controlPoints, seg.sampleStep, &m_chunkCenterECEF);
        seg.in500mChunk = true;
    } else {
        seg.splinePoints = RoadGenerator::generateSpline(seg.controlPoints, seg.sampleStep);
        if (seg.splinePoints.size() >= 2) {
            bool inChunk = false;
            for (const auto& sp : seg.splinePoints) {
                if (sp.pos.x >= minX && sp.pos.x <= maxX && sp.pos.z >= minZ && sp.pos.z <= maxZ) {
                    inChunk = true;
                    break;
                }
            }
            seg.in500mChunk = inChunk;
        }
    }

    if (seg.splinePoints.size() >= 2) {
        seg.length = seg.splinePoints.back().distance;
    }

    // Recompute setbacks and junction patches
    RoadGenerator::generateJunctionPatches(m_segments, m_junctions, m_isQuadsphereChunk ? &m_chunkCenterECEF : nullptr);

    // Rebuild ribbon geometry with setback trimming for touched segments
    std::vector<int> touchedSegs;
    touchedSegs.push_back(segIdx);
    if (seg.startJunctionId >= 0 && seg.startJunctionId < (int)m_junctions.size()) {
        for (const auto& conn : m_junctions[seg.startJunctionId].connections) {
            if (std::find(touchedSegs.begin(), touchedSegs.end(), conn.segmentIdx) == touchedSegs.end()) {
                touchedSegs.push_back(conn.segmentIdx);
            }
        }
    }
    if (seg.endJunctionId >= 0 && seg.endJunctionId < (int)m_junctions.size()) {
        for (const auto& conn : m_junctions[seg.endJunctionId].connections) {
            if (std::find(touchedSegs.begin(), touchedSegs.end(), conn.segmentIdx) == touchedSegs.end()) {
                touchedSegs.push_back(conn.segmentIdx);
            }
        }
    }

    for (int sIdx : touchedSegs) {
        if (sIdx < 0 || sIdx >= (int)m_segments.size()) continue;
        auto& s = m_segments[sIdx];
        auto trimmed = RoadGenerator::trimSpline(s.splinePoints, s.startSetback, s.endSetback);
        std::vector<Vertex> verts;
        std::vector<uint32_t> idxs;
        RoadGenerator::generateRibbonGeometry(trimmed, s.totalWidth,
                                              s.lanes, s.oneway, s.shoulderWidth,
                                              verts, idxs);
        RoadGenerator::clipRibbonAgainstJunctions(verts, s, m_junctions, m_segments);
        if (!verts.empty() && !idxs.empty()) {
            s.visualMesh = std::make_shared<Mesh>(verts.data(), (Uint32)verts.size(),
                                                  idxs.data(), (Uint32)idxs.size());
        }
    }

    if (segIdx == m_selectedSegmentIdx) {
        m_selectedMesh = seg.visualMesh;
    }

    m_mergedMesh = RoadGenerator::buildMergedMesh(m_segments, m_junctions, m_isQuadsphereChunk ? false : m_filter500mOnly, minX, maxX, minZ, maxZ);
}

void RoadEditor::export500mChunk(const std::string& path) {
    RoadBakeOptions opts;
    opts.outputPath = path;
    opts.filter500mOnly = true;
    opts.verbose = true;
    auto res = RoadBaker::bake(opts);
    if (res.success) {
        m_statusMessage = "Successfully baked 500m chunk to " + path + " (" + std::to_string(res.junctionCount) + " junctions)";
        m_statusTimer = 5.0f;
    } else {
        m_statusMessage = "Failed to export 500m chunk: " + res.errorMessage;
        m_statusTimer = 5.0f;
    }
}

void RoadEditor::loadCompiledChunk(const std::string& path) {
    std::vector<ProcessedRoadSegment> compiledSegs;
    std::shared_ptr<Mesh> compiledMesh;
    FRoadChunkHeader header;

    if (RoadBinary::loadChunk(path, compiledSegs, compiledMesh, header, &m_junctions)) {
        m_segments = compiledSegs;
        m_mergedMesh = compiledMesh;
        m_filter500mOnly = true;
        m_selectedSegmentIdx = -1;
        m_selectedMesh = nullptr;
        m_selectedControlPointIdx = -1;
        m_draggedControlPointIdx = -1;
        m_isDraggingControlPoint = false;
        m_statusMessage = "Loaded compiled " + path + " (" + std::to_string(compiledSegs.size()) + " segments, " + std::to_string(m_junctions.size()) + " junctions)";
        m_statusTimer = 5.0f;
    } else {
        m_statusMessage = "Failed to load " + path;
        m_statusTimer = 5.0f;
    }
}

bool RoadEditor::openChunkInputDialog(uint8_t& outFace, int& outX, int& outZ, bool& outIsQuadsphere) {
    std::string defaultAnswer;
    if (m_isQuadsphereChunk) {
        defaultAnswer = "Face " + std::to_string(m_activeFace) + " [" +
                        std::to_string(m_activeChunkX) + ", " +
                        std::to_string(m_activeChunkZ) + "]";
    } else {
        defaultAnswer = std::to_string(m_activeChunkX) + ", " + std::to_string(m_activeChunkZ);
    }

    std::string script =
        "osascript -e 'set dlg to display dialog "
        "\"Enter Chunk to Edit:\\n"
        "Formats:\\n"
        "  • Face 2 [16040, 20958] (Quadsphere L15)\\n"
        "  • 2, 16040, 20958\\n"
        "  • 16040, 20958 (defaults to Face 2)\\n"
        "  • chunk_2_16040_20958.froad\\n"
        "  • -1, 0 (Cartesian legacy)\" "
        "default answer \"" + defaultAnswer + "\" "
        "with title \"Select Road Chunk\" "
        "buttons {\"Cancel\", \"Open Chunk\"} default button \"Open Chunk\"' "
        "-e 'text returned of dlg' 2>/dev/null";

    FILE* pipe = popen(script.c_str(), "r");
    if (!pipe) return false;

    char buffer[512];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }
    int rc = pclose(pipe);
    if (rc != 0 || result.empty()) return false;

    while (!result.empty() && (result.back() == '\n' || result.back() == '\r' || result.back() == ' ' || result.back() == '\t')) {
        result.pop_back();
    }
    size_t start = result.find_first_not_of(" \t\r\n");
    if (start != std::string::npos) {
        result = result.substr(start);
    } else {
        return false;
    }

    std::string cleaned = result;
    size_t chunkPos = cleaned.find("chunk_");
    if (chunkPos != std::string::npos) {
        cleaned = cleaned.substr(chunkPos + 6);
    }
    size_t dotPos = cleaned.rfind(".froad");
    if (dotPos != std::string::npos) {
        cleaned = cleaned.substr(0, dotPos);
    }

    for (char& c : cleaned) {
        if (c == ',' || c == '[' || c == ']' || c == '(' || c == ')' || c == ':' || c == '_') {
            c = ' ';
        }
    }

    std::stringstream ss(cleaned);
    std::vector<int> numbers;
    std::string token;
    while (ss >> token) {
        if (token == "Face" || token == "face" || token == "Chunk" || token == "chunk" || token == "L15") {
            continue;
        }
        try {
            int val = std::stoi(token);
            numbers.push_back(val);
        } catch (...) {
        }
    }

    if (numbers.size() >= 3) {
        outFace = static_cast<uint8_t>(numbers[0]);
        outX = numbers[1];
        outZ = numbers[2];
        outIsQuadsphere = true;
        return true;
    } else if (numbers.size() == 2) {
        int a = numbers[0];
        int b = numbers[1];
        if (a > 100 || b > 100) {
            outFace = 2;
            outX = a;
            outZ = b;
            outIsQuadsphere = true;
            return true;
        }
        std::string qPath = m_chunksDir + "/chunk_2_" + std::to_string(a) + "_" + std::to_string(b) + ".froad";
        if (std::ifstream(qPath).good()) {
            outFace = 2;
            outX = a;
            outZ = b;
            outIsQuadsphere = true;
            return true;
        } else {
            outFace = 0;
            outX = a;
            outZ = b;
            outIsQuadsphere = false;
            return true;
        }
    }

    return false;
}

void RoadEditor::loadChunk(uint8_t face, int tx, int ty) {
    std::string path = m_chunksDir + "/chunk_" + std::to_string(face) + "_" +
                       std::to_string(tx) + "_" + std::to_string(ty) + ".froad";
    if (!std::ifstream(path).good()) {
        std::string cartPath = m_chunksDir + "/chunk_" + std::to_string(tx) + "_" + std::to_string(ty) + ".froad";
        if (std::ifstream(cartPath).good()) {
            loadChunkImpl(cartPath, 0, tx, ty, false);
            return;
        }
    }
    loadChunkImpl(path, face, tx, ty, true);
}

void RoadEditor::loadChunk(int cx, int cz) {
    std::string quadPath = m_chunksDir + "/chunk_" + std::to_string(m_activeFace) + "_" +
                           std::to_string(cx) + "_" + std::to_string(cz) + ".froad";
    std::string cartPath = m_chunksDir + "/chunk_" + std::to_string(cx) + "_" + std::to_string(cz) + ".froad";

    if (std::ifstream(quadPath).good() || (cx > 100 || cz > 100)) {
        loadChunk(m_activeFace, cx, cz);
    } else {
        loadChunkImpl(cartPath, 0, cx, cz, false);
    }
}

void RoadEditor::loadChunkImpl(const std::string& path, uint8_t face, int tx, int ty, bool isQuad) {
    m_activeFace = face;
    m_activeChunkX = tx;
    m_activeChunkZ = ty;
    m_isQuadsphereChunk = isQuad;

    std::vector<ProcessedRoadSegment> compiledSegs;
    std::shared_ptr<Mesh> compiledMesh;
    FRoadChunkHeader header;
    m_junctions.clear();
    m_adjacentMeshes.clear();

    bool loaded = RoadBinary::loadChunk(path, compiledSegs, compiledMesh, header, &m_junctions);
    if (loaded) {
        m_segments = compiledSegs;
        for (auto& s : m_segments) {
            s.in500mChunk = true;
        }
        m_mergedMesh = compiledMesh;
        m_filter500mOnly = true;
        m_selectedSegmentIdx = -1;
        m_selectedMesh = nullptr;
        m_selectedControlPointIdx = -1;
        m_draggedControlPointIdx = -1;
        m_isDraggingControlPoint = false;

        m_isQuadsphereChunk = (header.version >= 3);
        if (m_isQuadsphereChunk) {
            m_activeFace = header.face;
            m_activeLOD = header.lod > 0 ? header.lod : 15;
            m_activeChunkX = header.chunkX;
            m_activeChunkZ = header.chunkZ;
            m_chunkCenterECEF = glm::dvec3(header.chunkCenterECEF[0], header.chunkCenterECEF[1], header.chunkCenterECEF[2]);
            if (glm::length(m_chunkCenterECEF) < 1000.0) {
                double minU, maxU, minV, maxV;
                Quadsphere::getTileUVBounds(m_activeLOD, m_activeChunkX, m_activeChunkZ, minU, maxU, minV, maxV);
                m_chunkCenterECEF = Quadsphere::quadsphereToECEF(m_activeFace, (minU + maxU) * 0.5, (minV + maxV) * 0.5, 0.0);
            }
            double cLat, cLon, cAlt;
            Quadsphere::ecefToGeodetic(m_chunkCenterECEF, cLat, cLon, cAlt);
            m_R_ecef_to_enu = glm::mat4(Quadsphere::getECEFtoENU(cLat, cLon));
            m_R_enu_to_ecef = glm::mat4(Quadsphere::getENUtoECEF(cLat, cLon));

            // Load 8 adjacent Quadsphere chunks as read-only visual context
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int adjX = m_activeChunkX + dx;
                    int adjY = m_activeChunkZ + dy;
                    std::string adjPath = m_chunksDir + "/chunk_" + std::to_string(m_activeFace) + "_" +
                                          std::to_string(adjX) + "_" + std::to_string(adjY) + ".froad";
                    std::ifstream f(adjPath);
                    if (f.good()) {
                        f.close();
                        std::vector<ProcessedRoadSegment> dummySegs;
                        std::shared_ptr<Mesh> adjMesh;
                        FRoadChunkHeader dummyHdr;
                        if (RoadBinary::loadChunk(adjPath, dummySegs, adjMesh, dummyHdr, nullptr)) {
                            if (adjMesh && adjMesh->indexCount > 0) {
                                glm::dvec3 adjCenter(dummyHdr.chunkCenterECEF[0], dummyHdr.chunkCenterECEF[1], dummyHdr.chunkCenterECEF[2]);
                                if (glm::length(adjCenter) < 1000.0) {
                                    double aMinU, aMaxU, aMinV, aMaxV;
                                    Quadsphere::getTileUVBounds(m_activeLOD, adjX, adjY, aMinU, aMaxU, aMinV, aMaxV);
                                    adjCenter = Quadsphere::quadsphereToECEF(m_activeFace, (aMinU + aMaxU) * 0.5, (aMinV + aMaxV) * 0.5, 0.0);
                                }
                                glm::vec3 relCenterENU = glm::vec3(m_R_ecef_to_enu * glm::dvec4(adjCenter - m_chunkCenterECEF, 0.0));
                                glm::mat4 adjModel = glm::translate(glm::mat4(1.0f), relCenterENU) * m_R_ecef_to_enu;
                                m_adjacentMeshes.push_back({ adjMesh, adjModel });
                            }
                        }
                    }
                }
            }

            m_statusMessage = "Loaded Quadsphere chunk Face " + std::to_string(m_activeFace) + " [" +
                              std::to_string(m_activeChunkX) + ", " + std::to_string(m_activeChunkZ) + "]: " +
                              std::to_string(compiledSegs.size()) + " segments, " +
                              std::to_string(m_junctions.size()) + " junctions, " +
                              std::to_string(m_adjacentMeshes.size()) + " neighbors visible";
            m_statusTimer = 5.0f;
        } else {
            m_R_ecef_to_enu = glm::mat4(1.0f);
            m_R_enu_to_ecef = glm::mat4(1.0f);
            m_chunkCenterECEF = glm::dvec3(0.0);

            // Load 8 adjacent Cartesian chunks as read-only visual context
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dz == 0) continue;
                    std::string adjPath = m_chunksDir + "/chunk_" + std::to_string(m_activeChunkX + dx) + "_" + std::to_string(m_activeChunkZ + dz) + ".froad";
                    std::ifstream f(adjPath);
                    if (f.good()) {
                        f.close();
                        std::vector<ProcessedRoadSegment> dummySegs;
                        std::shared_ptr<Mesh> adjMesh;
                        FRoadChunkHeader dummyHdr;
                        if (RoadBinary::loadChunk(adjPath, dummySegs, adjMesh, dummyHdr, nullptr)) {
                            if (adjMesh && adjMesh->indexCount > 0) {
                                m_adjacentMeshes.push_back({ adjMesh, glm::mat4(1.0f) });
                            }
                        }
                    }
                }
            }

            m_statusMessage = "Loaded chunk (" + std::to_string(m_activeChunkX) + ", " + std::to_string(m_activeChunkZ) + "): " +
                              std::to_string(compiledSegs.size()) + " segments, " +
                              std::to_string(m_junctions.size()) + " junctions, " +
                              std::to_string(m_adjacentMeshes.size()) + " neighbor chunks visible";
            m_statusTimer = 5.0f;
        }
    } else {
        m_segments.clear();
        m_mergedMesh = nullptr;
        m_selectedSegmentIdx = -1;
        m_selectedMesh = nullptr;
        if (m_isQuadsphereChunk) {
            m_statusMessage = "Chunk Face " + std::to_string(m_activeFace) + " [" +
                              std::to_string(m_activeChunkX) + ", " + std::to_string(m_activeChunkZ) + "] not found: " + path;
        } else {
            m_statusMessage = "Chunk (" + std::to_string(m_activeChunkX) + ", " + std::to_string(m_activeChunkZ) + ") not found: " + path;
        }
        m_statusTimer = 5.0f;
    }

    // Rebuild terrain mesh for active chunk
    buildChunkTerrainMesh();

    // Center camera focus on active chunk center
    if (m_isQuadsphereChunk) {
        m_cameraFocus = glm::vec3(0.0f, 1.0f, 0.0f);
        m_topDownPos = glm::vec3(0.0f, 350.0f, 0.0f);
        m_orbitDistance = 220.0f;
        m_orbitPitch = 48.0f;
        m_orbitYaw = -25.0f;
    } else {
        float centerX = m_activeChunkX * 512.0f;
        float centerZ = m_activeChunkZ * 512.0f;
        float centerY = m_hasTerrain ? sampleTerrainHeight(centerX, centerZ) : 0.0f;
        m_cameraFocus = glm::vec3(centerX, centerY + 1.0f, centerZ);
        m_topDownPos = glm::vec3(centerX, centerY + 350.0f, centerZ);
    }
}

void RoadEditor::saveActiveChunk() {
    if (m_isQuadsphereChunk) {
        double minU, maxU, minV, maxV;
        Quadsphere::getTileUVBounds(m_activeLOD, m_activeChunkX, m_activeChunkZ, minU, maxU, minV, maxV);
        std::string path = m_chunksDir + "/chunk_" + std::to_string(m_activeFace) + "_" +
                           std::to_string(m_activeChunkX) + "_" + std::to_string(m_activeChunkZ) + ".froad";

        if (RoadBinary::saveQuadsphereChunk(path, m_activeFace, m_activeLOD, m_activeChunkX, m_activeChunkZ,
                                            m_chunkCenterECEF, (float)minU, (float)maxU, (float)minV, (float)maxV,
                                            m_segments, m_junctions)) {
            m_statusMessage = "Saved Quadsphere chunk " + path + " (" + std::to_string(m_segments.size()) + " segments, " + std::to_string(m_junctions.size()) + " junctions)";
            m_statusTimer = 5.0f;
        } else {
            m_statusMessage = "Failed to save " + path;
            m_statusTimer = 5.0f;
        }
        return;
    }

    float halfS = 256.0f;
    float minX = m_activeChunkX * 512.0f - halfS;
    float maxX = m_activeChunkX * 512.0f + halfS;
    float minZ = m_activeChunkZ * 512.0f - halfS;
    float maxZ = m_activeChunkZ * 512.0f + halfS;
    std::string path = m_chunksDir + "/chunk_" + std::to_string(m_activeChunkX) + "_" + std::to_string(m_activeChunkZ) + ".froad";

    if (RoadBinary::saveChunk(path, m_activeChunkX, m_activeChunkZ, minX, maxX, minZ, maxZ, m_segments, m_junctions)) {
        m_statusMessage = "Overwrote " + path + " (" + std::to_string(m_segments.size()) + " segments, " + std::to_string(m_junctions.size()) + " junctions)";
        m_statusTimer = 5.0f;
    } else {
        m_statusMessage = "Failed to save " + path;
        m_statusTimer = 5.0f;
    }
}

void RoadEditor::enterDriveMode(int segmentIndex) {
    m_viewMode = EditorViewMode::DriveTest;

    glm::vec3 spawnPos(0.0f, 0.05f, 0.0f);
    glm::vec3 spawnDir(0.0f, 0.0f, 1.0f);

    if (segmentIndex >= 0 && segmentIndex < (int)m_segments.size()) {
        const auto& seg = m_segments[segmentIndex];
        if (seg.splinePoints.size() >= 2) {
            spawnPos = seg.splinePoints[0].pos;
            spawnDir = seg.splinePoints[0].tangent;
        }
    } else if (!m_segments.empty()) {
        for (const auto& seg : m_segments) {
            if (seg.in500mChunk && seg.splinePoints.size() >= 2) {
                spawnPos = seg.splinePoints[0].pos;
                spawnDir = seg.splinePoints[0].tangent;
                break;
            }
        }
    }

    m_vehicle.reset(spawnPos, spawnDir);
    m_statusMessage = "Drive Mode active: WASD to drive, ESC to exit";
    m_statusTimer = 4.0f;
}

void RoadEditor::exitDriveMode() {
    m_viewMode = EditorViewMode::Orbit3D;
    m_cameraFocus = m_vehicle.getPosition();
}

void RoadEditor::handleEvent(const SDL_Event& event) {
    if (event.type == SDL_EVENT_MOUSE_MOTION) {
        m_mouseX = event.motion.x;
        m_mouseY = event.motion.y;
        m_mouseDeltaX = event.motion.xrel;
        m_mouseDeltaY = event.motion.yrel;
    } else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            if (m_activeTool == EditorTool::Pan) {
                m_mouseLeftDown = true;
                return;
            }

            // Check if user clicked on a control point handle of any road segment
            if (!m_isAddingJunction && m_viewMode != EditorViewMode::DriveTest && !UI::wantsMouseInput()) {
                int logW = 1280, logH = 800;
                if (m_engine && m_engine->getWindow()) {
                    SDL_GetWindowSize(m_engine->getWindow(), &logW, &logH);
                }
                glm::mat4 vp = getProjMatrix((float)logW / (float)logH) * getViewMatrix();

                int bestSeg = -1;
                int bestCp = -1;
                float bestDist2 = 1e9f;

                auto checkSegmentCPs = [&](int s) {
                    if (s < 0 || s >= (int)m_segments.size()) return;
                    const auto& seg = m_segments[s];
                    for (size_t i = 0; i < seg.controlPoints.size(); ++i) {
                        glm::vec4 cpWorld = m_isQuadsphereChunk ? (m_R_ecef_to_enu * glm::vec4(seg.controlPoints[i], 1.0f))
                                                                : glm::vec4(seg.controlPoints[i], 1.0f);
                        glm::vec4 clip = vp * cpWorld;
                        if (clip.w > 0.01f) {
                            glm::vec3 ndc = glm::vec3(clip) / clip.w;
                            if (ndc.z >= 0.0f && ndc.z <= 1.0f) {
                                float sx = (ndc.x * 0.5f + 0.5f) * (float)logW;
                                float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)logH;
                                float d2 = (m_mouseX - sx) * (m_mouseX - sx) + (m_mouseY - sy) * (m_mouseY - sy);
                                float hitRadius = 14.0f * m_uiScale;
                                if (d2 <= hitRadius * hitRadius && d2 < bestDist2) {
                                    bestDist2 = d2;
                                    bestSeg = s;
                                    bestCp = (int)i;
                                }
                            }
                        }
                    }
                };

                // Prioritize checking selected segment first
                if (m_selectedSegmentIdx >= 0) {
                    checkSegmentCPs(m_selectedSegmentIdx);
                }
                if (bestCp < 0) {
                    for (size_t s = 0; s < m_segments.size(); ++s) {
                        if ((int)s == m_selectedSegmentIdx) continue;
                        checkSegmentCPs((int)s);
                        if (bestCp >= 0) break;
                    }
                }

                if (bestSeg >= 0 && bestCp >= 0) {
                    const auto& hitSeg = m_segments[bestSeg];
                    bool isStart = (bestCp == 0);
                    bool isEnd = (bestCp == (int)hitSeg.controlPoints.size() - 1);
                    int juncId = isStart ? hitSeg.startJunctionId : (isEnd ? hitSeg.endJunctionId : -1);
                    bool isMultiJunc = (juncId >= 0 && juncId < (int)m_junctions.size() && m_junctions[juncId].connections.size() > 1);

                    // Clicking directly on a multi-way junction marker selects the junction and opens Junction Inspector!
                    if (isMultiJunc) {
                        m_selectedJunctionIdx = juncId;
                        m_selectedSegmentIdx = -1;
                        m_selectedControlPointIdx = -1;
                        m_draggedControlPointIdx = -1;
                        m_isDraggingControlPoint = false;
                        m_selectedMesh = m_junctions[juncId].patchMesh;
                        m_rightPanelTab = 2; // Switch to Junction Inspector tab
                        m_statusMessage = "Selected Junction #" + std::to_string(m_junctions[juncId].id) + " (" +
                                          std::to_string(m_junctions[juncId].connections.size()) + "-way)";
                        m_statusTimer = 3.5f;
                        m_mouseLeftDown = true;
                        return;
                    }

                    if (m_selectedSegmentIdx != bestSeg) {
                        m_selectedSegmentIdx = bestSeg;
                        m_selectedJunctionIdx = -1;
                        m_selectedMesh = m_segments[bestSeg].visualMesh;
                    }
                    m_draggedControlPointIdx = bestCp;
                    m_selectedControlPointIdx = bestCp;
                    m_isDraggingControlPoint = true;
                    m_dragStartMouseScreen = glm::vec2(m_mouseX, m_mouseY);
                    m_dragStartControlPointPos = m_segments[bestSeg].controlPoints[bestCp];
                    float curY = 0.05f;
                    glm::vec3 cpWorld = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(m_segments[bestSeg].controlPoints[bestCp], 1.0f))
                                                            : m_segments[bestSeg].controlPoints[bestCp];
                    curY = cpWorld.y;
                    m_dragStartGroundHit = getGroundRayHit(m_mouseX, m_mouseY, logW, logH, curY);
                    m_dragThresholdPassed = false;
                    m_mouseLeftDown = true;
                    return;
                }
            }

            m_mouseLeftDown = true;
            m_mouseLeftClicked = true;
        } else if (event.button.button == SDL_BUTTON_MIDDLE) {
            // Rebound orbit to Middle Mouse Button
            m_mouseMiddleDown = true;
            if (m_viewMode == EditorViewMode::Orbit3D && !UI::wantsMouseInput()) {
                recenterOrbitOnTerrain();
            }
        } else if (event.button.button == SDL_BUTTON_RIGHT) {
            m_mouseRightDown = true;
        }
    } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            m_mouseLeftDown = false;
            m_isDraggingControlPoint = false;
            m_draggedControlPointIdx = -1;
            m_dragThresholdPassed = false;
        } else if (event.button.button == SDL_BUTTON_MIDDLE) {
            m_mouseMiddleDown = false;
        } else if (event.button.button == SDL_BUTTON_RIGHT) {
            m_mouseRightDown = false;
        }
    } else if (event.type == SDL_EVENT_MOUSE_WHEEL) {
        m_mouseWheel = event.wheel.y;
    } else if (event.type == SDL_EVENT_KEY_DOWN) {
        if (event.key.scancode < SDL_SCANCODE_COUNT) {
            m_keyState[event.key.scancode] = true;
        }
        if (!event.key.repeat) {
            if (event.key.key == SDLK_ESCAPE) {
                if (m_viewMode == EditorViewMode::DriveTest) {
                    exitDriveMode();
                } else if (m_isAddingJunction) {
                    m_isAddingJunction = false;
                    m_pendingJunctionConns.clear();
                    m_statusMessage = "Cancelled junction creation.";
                    m_statusTimer = 3.0f;
                } else if (m_selectedControlPointIdx >= 0) {
                    m_selectedControlPointIdx = -1;
                } else if (m_selectedSegmentIdx >= 0) {
                    m_selectedSegmentIdx = -1;
                    m_selectedMesh = nullptr;
                } else if (m_selectedJunctionIdx >= 0) {
                    m_selectedJunctionIdx = -1;
                    m_selectedMesh = nullptr;
                }
            } else if (event.key.key == SDLK_RETURN || event.key.key == SDLK_KP_ENTER) {
                if (m_isAddingJunction) {
                    createJunctionFromPending();
                }
            } else if (event.key.key == SDLK_TAB) {
                if (m_viewMode == EditorViewMode::Orbit3D) m_viewMode = EditorViewMode::TopDown2D;
                else if (m_viewMode == EditorViewMode::TopDown2D) m_viewMode = EditorViewMode::Orbit3D;
                else exitDriveMode();
            } else if (event.key.key == SDLK_SPACE && m_viewMode != EditorViewMode::DriveTest) {
                enterDriveMode(m_selectedSegmentIdx);
            } else if ((event.key.key == SDLK_DELETE || event.key.key == SDLK_BACKSPACE) && m_viewMode != EditorViewMode::DriveTest) {
                if (m_selectedControlPointIdx >= 0 && m_selectedSegmentIdx >= 0 && m_segments[m_selectedSegmentIdx].controlPoints.size() > 2) {
                    m_segments[m_selectedSegmentIdx].controlPoints.erase(m_segments[m_selectedSegmentIdx].controlPoints.begin() + m_selectedControlPointIdx);
                    m_selectedControlPointIdx = -1;
                    rebuildSegmentGeometry(m_selectedSegmentIdx);
                    m_statusMessage = "Deleted control point.";
                    m_statusTimer = 2.5f;
                } else if (m_selectedSegmentIdx >= 0) {
                    deleteSegment(m_selectedSegmentIdx);
                } else if (m_selectedJunctionIdx >= 0) {
                    removeJunction(m_selectedJunctionIdx);
                }
            } else if (event.key.key == SDLK_C && m_viewMode != EditorViewMode::DriveTest) {
                uint8_t face = m_activeFace;
                int x = m_activeChunkX, z = m_activeChunkZ;
                bool isQuad = m_isQuadsphereChunk;
                if (openChunkInputDialog(face, x, z, isQuad)) {
                    if (isQuad) loadChunk(face, x, z);
                    else loadChunk(x, z);
                }
            }
        }
    } else if (event.type == SDL_EVENT_KEY_UP) {
        if (event.key.scancode < SDL_SCANCODE_COUNT) {
            m_keyState[event.key.scancode] = false;
        }
    }
}

glm::vec3 RoadEditor::getGroundRayHit(float mx, float my, int screenW, int screenH, float planeY) const {
    glm::mat4 view = getViewMatrix();
    glm::mat4 proj = getProjMatrix((float)screenW / (float)screenH);
    glm::mat4 invVP = glm::inverse(proj * view);

    float ndcX = (mx / (float)screenW) * 2.0f - 1.0f;
    float ndcY = 1.0f - (my / (float)screenH) * 2.0f;

    glm::vec4 nearPt = invVP * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
    glm::vec4 farPt  = invVP * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
    nearPt /= nearPt.w;
    farPt /= farPt.w;

    glm::vec3 rayOrigin = glm::vec3(nearPt);
    glm::vec3 rayDir = glm::normalize(glm::vec3(farPt - nearPt));

    if (std::abs(rayDir.y) < 1e-4f) return glm::vec3(rayOrigin.x, planeY, rayOrigin.z);
    float t = (planeY - rayOrigin.y) / rayDir.y;
    return rayOrigin + rayDir * t;
}

void RoadEditor::update(float dt) {
    if (m_statusTimer > 0.0f) {
        m_statusTimer -= dt;
    }

    const bool* sdlKeys = SDL_GetKeyboardState(nullptr);
    bool combinedKeys[SDL_SCANCODE_COUNT];
    for (int i = 0; i < SDL_SCANCODE_COUNT; ++i) {
        combinedKeys[i] = (sdlKeys && sdlKeys[i]) || m_keyState[i];
    }
    const bool* keys = combinedKeys;

    if (m_viewMode == EditorViewMode::DriveTest) {
        m_vehicle.update(dt, keys, m_segments, m_junctions);
    } else {
        int logW = 1280, logH = 800;
        if (m_engine && m_engine->getWindow()) {
            SDL_GetWindowSize(m_engine->getWindow(), &logW, &logH);
        }

        // 1. Control Point Dragging (with threshold to prevent unintentional movement on click)
        if (m_isDraggingControlPoint && m_selectedSegmentIdx >= 0 &&
            m_selectedSegmentIdx < (int)m_segments.size() && m_draggedControlPointIdx >= 0) {
            auto& seg = m_segments[m_selectedSegmentIdx];
            if (m_draggedControlPointIdx < (int)seg.controlPoints.size()) {
                float mouseDist = glm::distance(glm::vec2(m_mouseX, m_mouseY), m_dragStartMouseScreen);
                if (!m_dragThresholdPassed && mouseDist > 5.0f) {
                    m_dragThresholdPassed = true;
                }

                if (m_dragThresholdPassed) {
                    float curY = 0.05f;
                    glm::vec3 cpWorld = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(m_dragStartControlPointPos, 1.0f))
                                                            : m_dragStartControlPointPos;
                    curY = cpWorld.y;
                    glm::vec3 curHit = getGroundRayHit(m_mouseX, m_mouseY, logW, logH, curY);
                    glm::vec3 deltaHit = curHit - m_dragStartGroundHit;

                    glm::vec3 initENU = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(m_dragStartControlPointPos, 1.0f))
                                                            : m_dragStartControlPointPos;
                    glm::vec3 newPosENU = initENU + glm::vec3(deltaHit.x, 0.0f, deltaHit.z);
                    glm::vec3 newPos = m_isQuadsphereChunk ? glm::vec3(m_R_enu_to_ecef * glm::vec4(newPosENU, 1.0f)) : newPosENU;

                    bool isStart = (m_draggedControlPointIdx == 0);
                    bool isEnd = (m_draggedControlPointIdx == (int)seg.controlPoints.size() - 1);
                    int juncId = isStart ? seg.startJunctionId : (isEnd ? seg.endJunctionId : -1);

                    if (juncId >= 0 && juncId < (int)m_junctions.size()) {
                        auto& junc = m_junctions[juncId];
                        junc.position = newPos;
                        // Update all connected road segment endpoints simultaneously
                        for (const auto& conn : junc.connections) {
                            if (conn.segmentIdx >= 0 && conn.segmentIdx < (int)m_segments.size()) {
                                auto& connSeg = m_segments[conn.segmentIdx];
                                if (conn.isStart && !connSeg.controlPoints.empty()) {
                                    connSeg.controlPoints.front() = newPos;
                                } else if (!conn.isStart && !connSeg.controlPoints.empty()) {
                                    connSeg.controlPoints.back() = newPos;
                                }
                                rebuildSegmentGeometry(conn.segmentIdx);
                            }
                        }
                    } else {
                        seg.controlPoints[m_draggedControlPointIdx] = newPos;
                        rebuildSegmentGeometry(m_selectedSegmentIdx);
                    }
                }
            }
        }
        // 2. Camera Controls (when not dragging control point or interacting with UI)
        else if (!UI::wantsMouseInput()) {
            if (m_viewMode == EditorViewMode::Orbit3D) {
                // Rebound Orbit to Middle Mouse Button (inverted horizontal axis for natural tumbling)
                if (m_mouseMiddleDown) {
                    m_orbitYaw -= m_mouseDeltaX * 0.35f;
                    m_orbitPitch += m_mouseDeltaY * 0.35f;
                    m_orbitPitch = std::clamp(m_orbitPitch, 5.0f, 88.0f);
                }

                // Compute exact ground-plane camera forward and right vectors
                float radYaw = glm::radians(m_orbitYaw);
                float radPitch = glm::radians(m_orbitPitch);
                glm::vec3 camOffset(
                    m_orbitDistance * std::cos(radPitch) * std::sin(radYaw),
                    m_orbitDistance * std::sin(radPitch),
                    m_orbitDistance * std::cos(radPitch) * std::cos(radYaw)
                );
                glm::vec3 camPos = m_cameraFocus + camOffset;
                glm::vec3 camFwd = glm::normalize(glm::vec3(m_cameraFocus.x - camPos.x, 0.0f, m_cameraFocus.z - camPos.z));
                // Right-handed coordinate cross product: Up(0,1,0) x camFwd gives screen-right
                glm::vec3 camRight = glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), camFwd));

                // Panning: Right-click drag anytime, or either mouse button when Pan tool is active
                bool isPanning = m_mouseRightDown || (m_activeTool == EditorTool::Pan && m_mouseLeftDown);
                if (isPanning) {
                    float panSpeed = m_orbitDistance * 0.0018f;
                    m_cameraFocus += camRight * (m_mouseDeltaX * panSpeed);
                    m_cameraFocus += camFwd * (m_mouseDeltaY * panSpeed);
                }

                if (std::abs(m_mouseWheel) > 0.01f) {
                    m_orbitDistance -= m_mouseWheel * 25.0f;
                    m_orbitDistance = std::clamp(m_orbitDistance, 10.0f, 1500.0f);
                }

                // Keyboard zoom: Z zooms in, X zooms out
                float keyZoomSpeed = m_orbitDistance * 1.5f * dt;
                if (keys[SDL_SCANCODE_Z]) {
                    m_orbitDistance -= keyZoomSpeed;
                    m_orbitDistance = std::clamp(m_orbitDistance, 10.0f, 1500.0f);
                }
                if (keys[SDL_SCANCODE_X]) {
                    m_orbitDistance += keyZoomSpeed;
                    m_orbitDistance = std::clamp(m_orbitDistance, 10.0f, 1500.0f);
                }

                // Keyboard panning: D moves view right, A moves view left, W moves view forward, S moves view backward
                float keyPanSpeed = m_orbitDistance * 0.8f * dt;
                if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) m_cameraFocus += camFwd * keyPanSpeed;
                if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) m_cameraFocus -= camFwd * keyPanSpeed;
                if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) m_cameraFocus += camRight * keyPanSpeed; // Moves view left
                if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) m_cameraFocus -= camRight * keyPanSpeed; // Moves view right
            } else if (m_viewMode == EditorViewMode::TopDown2D) {
                bool isPanning = m_mouseRightDown || m_mouseMiddleDown || (m_activeTool == EditorTool::Pan && m_mouseLeftDown);
                if (isPanning) {
                    float panSpeed = (m_topDownPos.y / 600.0f);
                    m_topDownPos.x -= m_mouseDeltaX * panSpeed;
                    m_topDownPos.z += m_mouseDeltaY * panSpeed;
                }

                if (std::abs(m_mouseWheel) > 0.01f) {
                    m_topDownPos.y -= m_mouseWheel * 35.0f;
                    m_topDownPos.y = std::clamp(m_topDownPos.y, 20.0f, 2000.0f);
                }

                // Keyboard zoom: Z zooms in, X zooms out
                float keyZoomSpeed = m_topDownPos.y * 1.5f * dt;
                if (keys[SDL_SCANCODE_Z]) {
                    m_topDownPos.y -= keyZoomSpeed;
                    m_topDownPos.y = std::clamp(m_topDownPos.y, 20.0f, 2000.0f);
                }
                if (keys[SDL_SCANCODE_X]) {
                    m_topDownPos.y += keyZoomSpeed;
                    m_topDownPos.y = std::clamp(m_topDownPos.y, 20.0f, 2000.0f);
                }

                float panSpeed = m_topDownPos.y * 0.8f * dt;
                if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) m_topDownPos.z -= panSpeed;
                if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) m_topDownPos.z += panSpeed;
                if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) m_topDownPos.x -= panSpeed; // Moves view left
                if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) m_topDownPos.x += panSpeed; // Moves view right
            }

            // Click selection in 3D viewport (active only when Select tool is chosen)
            if (m_mouseLeftClicked && m_activeTool == EditorTool::Select && m_viewMode != EditorViewMode::DriveTest && !m_isDraggingControlPoint && (!m_enableInternalUI || !UI::wantsMouseInput())) {
                if (m_isAddingJunction) {
                    handleJunctionSelectionClick(m_mouseX, m_mouseY, logW, logH);
                } else {
                    raycastSelectRoad(m_mouseX, m_mouseY, logW, logH);
                }
            }
        }
    }

    m_mouseDeltaX = 0.0f;
    m_mouseDeltaY = 0.0f;
    m_mouseWheel = 0.0f;
}

void RoadEditor::raycastSelectRoad(float mx, float my, int screenW, int screenH) {
    if (m_enableInternalUI) {
        // Avoid clicking on top bar or side panels in standalone immediate mode
        if (my < 45.0f * m_uiScale) return;
        if (mx < 310.0f * m_uiScale) return;
        if (mx > (float)screenW - 350.0f * m_uiScale) return;
    }

    glm::mat4 view = getViewMatrix();
    glm::mat4 proj = getProjMatrix((float)screenW / (float)screenH);
    glm::mat4 invVP = glm::inverse(proj * view);

    float ndcX = (mx / (float)screenW) * 2.0f - 1.0f;
    float ndcY = 1.0f - (my / (float)screenH) * 2.0f;

    glm::vec4 nearPt = invVP * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
    glm::vec4 farPt  = invVP * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
    nearPt /= nearPt.w;
    farPt /= farPt.w;

    glm::vec3 rayOrigin = glm::vec3(nearPt);
    glm::vec3 rayDir = glm::normalize(glm::vec3(farPt - nearPt));

    int bestSegIdx = -1;
    int bestJuncIdx = -1;
    float bestT = 1e9f;

    // 1. Raycast against road segments
    for (size_t i = 0; i < m_segments.size(); ++i) {
        const auto& seg = m_segments[i];
        if (m_filter500mOnly && !seg.in500mChunk) continue;

        float maxD = seg.totalWidth * 0.75f;
        for (const auto& sp : seg.splinePoints) {
            glm::vec3 spPos = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(sp.pos, 1.0f)) : sp.pos;
            glm::vec3 diff = spPos - rayOrigin;
            float t = glm::dot(diff, rayDir);
            if (t > 0.0f && t < bestT) {
                glm::vec3 ptOnRay = rayOrigin + rayDir * t;
                float d3d = glm::distance(ptOnRay, spPos);
                if (d3d <= maxD) {
                    bestT = t;
                    bestSegIdx = (int)i;
                }
            }
        }
    }

    // 2. Raycast against junction patch plate meshes and center proximity
    for (size_t j = 0; j < m_junctions.size(); ++j) {
        const auto& junc = m_junctions[j];
        glm::vec3 juncPos = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(junc.position, 1.0f)) : junc.position;

        // Proximity test to junction center
        glm::vec3 diff = juncPos - rayOrigin;
        float t = glm::dot(diff, rayDir);
        if (t > 0.0f) {
            glm::vec3 ptOnRay = rayOrigin + rayDir * t;
            float d3d = glm::distance(ptOnRay, juncPos);
            float juncHitRadius = 9.0f;
            if (d3d <= juncHitRadius && t < bestT) {
                bestT = t;
                bestJuncIdx = (int)j;
                bestSegIdx = -1;
            }
        }

        if (junc.patchIndices.empty() || junc.patchVertices.empty()) continue;

        for (size_t t = 0; t + 2 < junc.patchIndices.size(); t += 3) {
            glm::vec3 v0(junc.patchVertices[junc.patchIndices[t + 0]].position[0],
                         junc.patchVertices[junc.patchIndices[t + 0]].position[1],
                         junc.patchVertices[junc.patchIndices[t + 0]].position[2]);
            glm::vec3 v1(junc.patchVertices[junc.patchIndices[t + 1]].position[0],
                         junc.patchVertices[junc.patchIndices[t + 1]].position[1],
                         junc.patchVertices[junc.patchIndices[t + 1]].position[2]);
            glm::vec3 v2(junc.patchVertices[junc.patchIndices[t + 2]].position[0],
                         junc.patchVertices[junc.patchIndices[t + 2]].position[1],
                         junc.patchVertices[junc.patchIndices[t + 2]].position[2]);

            if (m_isQuadsphereChunk) {
                v0 = glm::vec3(m_R_ecef_to_enu * glm::vec4(v0, 1.0f));
                v1 = glm::vec3(m_R_ecef_to_enu * glm::vec4(v1, 1.0f));
                v2 = glm::vec3(m_R_ecef_to_enu * glm::vec4(v2, 1.0f));
            }

            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            glm::vec3 h = glm::cross(rayDir, edge2);
            float a = glm::dot(edge1, h);
            if (std::abs(a) < 1e-6f) continue;
            float f = 1.0f / a;
            glm::vec3 s = rayOrigin - v0;
            float u = f * glm::dot(s, h);
            if (u < 0.0f || u > 1.0f) continue;
            glm::vec3 q = glm::cross(s, edge1);
            float v = f * glm::dot(rayDir, q);
            if (v < 0.0f || u + v > 1.0f) continue;
            float rayT = f * glm::dot(edge2, q);
            if (rayT > 0.0f && rayT < bestT) {
                bestT = rayT;
                bestJuncIdx = (int)j;
                bestSegIdx = -1;
            }
        }
    }

    m_selectedControlPointIdx = -1;
    m_draggedControlPointIdx = -1;
    m_isDraggingControlPoint = false;

    if (bestJuncIdx >= 0) {
        m_selectedJunctionIdx = bestJuncIdx;
        m_selectedSegmentIdx = -1;
        m_selectedMesh = m_junctions[bestJuncIdx].patchMesh;
        m_rightPanelTab = 2; // Switch directly to Junction Inspector!
        m_statusMessage = "Selected Junction #" + std::to_string(m_junctions[bestJuncIdx].id) + " (" +
                          std::to_string(m_junctions[bestJuncIdx].connections.size()) + "-way)";
        m_statusTimer = 3.5f;
    } else if (bestSegIdx >= 0) {
        m_selectedSegmentIdx = bestSegIdx;
        m_selectedJunctionIdx = -1;
        m_selectedMesh = m_segments[bestSegIdx].visualMesh;
        m_rightPanelTab = 1; // Switch to Inspector tab when selecting a road
        std::string layerInfo = "Layer " + std::to_string(m_segments[bestSegIdx].layer);
        if (m_segments[bestSegIdx].isBridge) layerInfo += " [Bridge]";
        m_statusMessage = "Selected road: " + m_segments[bestSegIdx].name + " (" + layerInfo + ", " + m_segments[bestSegIdx].widthSource + ")";
        m_statusTimer = 3.5f;
    } else {
        m_selectedSegmentIdx = -1;
        m_selectedJunctionIdx = -1;
        m_selectedMesh = nullptr;
    }
}

glm::mat4 RoadEditor::getViewMatrix() const {
    if (m_viewMode == EditorViewMode::DriveTest) {
        return m_vehicle.getCameraViewMatrix();
    } else if (m_viewMode == EditorViewMode::TopDown2D) {
        return glm::lookAt(m_topDownPos,
                           glm::vec3(m_topDownPos.x, 0.0f, m_topDownPos.z),
                           glm::vec3(0.0f, 0.0f, -1.0f));
    } else {
        float radYaw = glm::radians(m_orbitYaw);
        float radPitch = glm::radians(m_orbitPitch);

        glm::vec3 offset(
            m_orbitDistance * std::cos(radPitch) * std::sin(radYaw),
            m_orbitDistance * std::sin(radPitch),
            m_orbitDistance * std::cos(radPitch) * std::cos(radYaw)
        );

        return glm::lookAt(m_cameraFocus + offset, m_cameraFocus, glm::vec3(0.0f, 1.0f, 0.0f));
    }
}

glm::mat4 RoadEditor::getProjMatrix(float aspect) const {
    return glm::perspectiveRH_ZO(glm::radians(55.0f), aspect, 0.2f, 3000.0f);
}

void RoadEditor::render(Engine& engine) {
    auto& rhi = RHI_Device::getInstance();
    SDL_Window* window = engine.getWindow();
    if (!window || !rhi.getDevice()) return;

    int logW = 1280, logH = 800;
    SDL_GetWindowSize(window, &logW, &logH);
    int pixW = 1280, pixH = 800;
    SDL_GetWindowSizeInPixels(window, &pixW, &pixH);

    float aspect = (float)pixW / (float)pixH;

    SDL_GPUTexture* swapchainTexture = nullptr;
    SDL_GPUCommandBuffer* cmdBuf = rhi.beginFrame(swapchainTexture);
    if (!cmdBuf || !swapchainTexture) return;

    // 1. Create Transient Depth Attachment at physical pixel resolution
    SDL_GPUTextureCreateInfo depthInfo = {};
    depthInfo.type = SDL_GPU_TEXTURETYPE_2D;
    depthInfo.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    depthInfo.width = pixW;
    depthInfo.height = pixH;
    depthInfo.layer_count_or_depth = 1;
    depthInfo.num_levels = 1;
    depthInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
    depthInfo.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

    SDL_GPUTexture* depthTexture = SDL_CreateGPUTexture(rhi.getDevice(), &depthInfo);

    // 2. PASS 1: 3D Scene Pass (Grid + Roads + Vehicle)
    SDL_GPUColorTargetInfo colorTarget = {};
    colorTarget.texture = swapchainTexture;
    colorTarget.clear_color = { 0.22f, 0.23f, 0.25f, 1.0f }; // Neutral CAD gray canvas
    colorTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    colorTarget.store_op = SDL_GPU_STOREOP_STORE;

    SDL_GPUDepthStencilTargetInfo depthTarget = {};
    depthTarget.texture = depthTexture;
    depthTarget.cycle = false;
    depthTarget.clear_depth = 1.0f;
    depthTarget.load_op = SDL_GPU_LOADOP_CLEAR;
    depthTarget.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depthTarget.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depthTarget.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmdBuf, &colorTarget, 1, &depthTarget);
    if (pass) {
        glm::mat4 view = getViewMatrix();
        glm::mat4 proj = getProjMatrix(aspect);

        // A. Draw 1m Grid
        if (m_showGrid && m_gridMesh && m_gridPipeline) {
            SDL_BindGPUGraphicsPipeline(pass, m_gridPipeline->getPipeline());

            struct GridUniforms {
                glm::mat4 model;
                glm::mat4 view;
                glm::mat4 projection;
                glm::vec4 viewPos;
                glm::vec4 gridParams;
            } gu = {};
            gu.model = glm::mat4(1.0f);
            gu.view = view;
            gu.projection = proj;
            gu.viewPos = glm::vec4(m_cameraFocus, 1.0f);
            gu.gridParams = glm::vec4(m_isQuadsphereChunk ? 0.0f : (m_activeChunkX * 512.0f),
                                      m_isQuadsphereChunk ? 0.0f : (m_activeChunkZ * 512.0f),
                                      256.0f, 0.0f);

            SDL_PushGPUVertexUniformData(cmdBuf, 0, &gu, sizeof(gu));
            SDL_PushGPUFragmentUniformData(cmdBuf, 0, &gu, sizeof(gu));

            SDL_GPUBufferBinding vb = { m_gridMesh->vertexBuffer.getBuffer(), 0 };
            SDL_BindGPUVertexBuffers(pass, 0, &vb, 1);
            SDL_GPUBufferBinding ib = { m_gridMesh->indexBuffer.getBuffer(), 0 };
            SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_32BIT);

            SDL_DrawGPUIndexedPrimitives(pass, m_gridMesh->indexCount, 1, 0, 0, 0);
        }

        // B. Draw Road Geometry
        if (m_roadRenderer.isInitialized()) {
            glm::mat4 activeModel = m_isQuadsphereChunk ? m_R_ecef_to_enu : glm::mat4(1.0f);

            // 1. Draw Active Editable Road Chunk (with occluded Mario-style silhouette)
            if (m_mergedMesh) {
                m_roadRenderer.drawMesh(pass, cmdBuf, view, proj, m_cameraFocus, *m_mergedMesh, glm::vec4(0.0f), activeModel, true);
            }

            // 2. Draw 8 Adjacent Chunks (Read-only visual context, non-editable)
            for (const auto& adj : m_adjacentMeshes) {
                if (adj.mesh && adj.mesh->indexCount > 0) {
                    m_roadRenderer.drawMesh(pass, cmdBuf, view, proj, m_cameraFocus, *adj.mesh, glm::vec4(0.0f), adj.modelMatrix, true);
                }
            }

            // Highlight selected road segment if any
            if (m_selectedMesh) {
                m_roadRenderer.drawMesh(pass, cmdBuf, view, proj, m_cameraFocus, *m_selectedMesh, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f), activeModel, true);
            }
        }

        // C. Draw Test Vehicle in Drive Mode
        if (m_viewMode == EditorViewMode::DriveTest && m_vehicle.getModel()) {
            auto carModel = m_vehicle.getModel();
            if (!carModel->meshes.empty()) {
                auto carMesh = carModel->meshes[0];

                struct CarUniforms {
                    glm::mat4 model;
                    glm::mat4 view;
                    glm::mat4 projection;
                    glm::vec4 viewPos;
                    glm::vec4 roadParams;
                } cu = {};
                cu.model = m_vehicle.getTransform();
                cu.view = view;
                cu.projection = proj;
                cu.viewPos = glm::vec4(m_vehicle.getPosition(), 1.0f);
                cu.roadParams = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

                SDL_PushGPUVertexUniformData(cmdBuf, 0, &cu, sizeof(cu));
                SDL_PushGPUFragmentUniformData(cmdBuf, 0, &cu, sizeof(cu));

                SDL_GPUBufferBinding vb = { carMesh->vertexBuffer.getBuffer(), 0 };
                SDL_BindGPUVertexBuffers(pass, 0, &vb, 1);
                SDL_GPUBufferBinding ib = { carMesh->indexBuffer.getBuffer(), 0 };
                SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_32BIT);

                SDL_DrawGPUIndexedPrimitives(pass, carMesh->indexCount, 1, 0, 0, 0);
            }
        }

        SDL_EndGPURenderPass(pass);
    }

    SDL_ReleaseGPUTexture(rhi.getDevice(), depthTexture);

    // 3. PASS 2: UI & Control Points Pass
    UI::beginFrame((float)logW, (float)logH, m_mouseX, m_mouseY,
                   m_mouseLeftDown, m_mouseLeftClicked, m_mouseWheel);
    m_mouseLeftClicked = false;

    // Draw control point handles for selected road segment
    drawControlPoints(logW, logH);

    // Draw immediate-mode UI windows and panels if enabled (standalone mode)
    if (m_enableInternalUI) {
        renderUI(logW, logH);
    }

    UI::endFrame();
    UI::render(cmdBuf, swapchainTexture);

    // 4. Submit Frame
    rhi.endFrame(cmdBuf);
}

void RoadEditor::drawControlPoints(int screenW, int screenH) {
    if (m_viewMode == EditorViewMode::DriveTest) return;

    // A. Draw Pending Junction Indicators & Connecting Spider-Web Lines
    if (m_isAddingJunction) {
        glm::mat4 view = getViewMatrix();
        glm::mat4 proj = getProjMatrix((float)screenW / (float)screenH);
        glm::mat4 vp = proj * view;

        // 1. Draw candidate endpoint markers on all road ends so user can clearly see what to click
        for (size_t s = 0; s < m_segments.size(); ++s) {
            const auto& seg = m_segments[s];
            if (seg.controlPoints.size() < 2) continue;
            if (!m_isQuadsphereChunk && m_filter500mOnly && !seg.in500mChunk) continue;

            auto drawCandidate = [&](const glm::vec3& rawPt, bool isStart) {
                // Check if already in pending list
                bool isSelected = false;
                for (const auto& conn : m_pendingJunctionConns) {
                    if (conn.segmentIdx == (int)s && conn.isStart == isStart) {
                        isSelected = true;
                        break;
                    }
                }
                if (isSelected) return; // Selected ones are drawn below in the pending list

                glm::vec3 pt = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(rawPt, 1.0f)) : rawPt;
                glm::vec4 clip = vp * glm::vec4(pt, 1.0f);
                if (clip.w > 0.01f) {
                    glm::vec3 ndc = glm::vec3(clip) / clip.w;
                    if (ndc.z >= 0.0f && ndc.z <= 1.0f) {
                        float sx = (ndc.x * 0.5f + 0.5f) * (float)screenW;
                        float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)screenH;
                        float dMouse = glm::distance(glm::vec2(m_mouseX, m_mouseY), glm::vec2(sx, sy));
                        bool isHovered = (dMouse <= 18.0f * m_uiScale);

                        if (isHovered) {
                            float r = 9.0f * m_uiScale;
                            UI::drawRect(sx - r, sy - r, r * 2.0f, r * 2.0f, glm::vec4(1.0f, 0.8f, 0.2f, 0.7f));
                            UI::drawRectOutline(sx - r, sy - r, r * 2.0f, r * 2.0f, 2.0f, glm::vec4(1.0f, 1.0f, 0.4f, 1.0f));
                            std::string tip = "Click to select: " + seg.name + (isStart ? " [Start]" : " [End]");
                            UI::drawText(tip, sx + r + 6.0f * m_uiScale, sy - 6.0f * m_uiScale, 0.85f * m_uiScale, glm::vec4(1.0f, 0.95f, 0.4f, 1.0f));
                        } else {
                            float r = 6.0f * m_uiScale;
                            UI::drawRect(sx - r, sy - r, r * 2.0f, r * 2.0f, glm::vec4(1.0f, 0.65f, 0.15f, 0.4f));
                            UI::drawRectOutline(sx - r, sy - r, r * 2.0f, r * 2.0f, 1.5f, glm::vec4(1.0f, 0.8f, 0.25f, 0.85f));
                        }
                    }
                }
            };

            drawCandidate(seg.controlPoints.front(), true);
            drawCandidate(seg.controlPoints.back(), false);
        }

        // 2. Draw selected endpoints and connecting lines
        glm::vec3 centroid(0.0f);
        int validCount = 0;
        std::vector<glm::vec2> screenEnds;

        for (size_t i = 0; i < m_pendingJunctionConns.size(); ++i) {
            const auto& conn = m_pendingJunctionConns[i];
            if (conn.segmentIdx < 0 || conn.segmentIdx >= (int)m_segments.size()) continue;
            const auto& seg = m_segments[conn.segmentIdx];
            if (seg.controlPoints.empty()) continue;

            glm::vec3 pt = conn.isStart ? seg.controlPoints.front() : seg.controlPoints.back();
            if (m_isQuadsphereChunk) pt = glm::vec3(m_R_ecef_to_enu * glm::vec4(pt, 1.0f));
            centroid += pt;
            validCount++;

            glm::vec4 clip = vp * glm::vec4(pt, 1.0f);
            if (clip.w > 0.01f) {
                glm::vec3 ndc = glm::vec3(clip) / clip.w;
                if (ndc.z >= 0.0f && ndc.z <= 1.0f) {
                    float sx = (ndc.x * 0.5f + 0.5f) * (float)screenW;
                    float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)screenH;
                    screenEnds.push_back(glm::vec2(sx, sy));

                    float sz = 20.0f * m_uiScale;
                    UI::drawRect(sx - sz * 0.5f, sy - sz * 0.5f, sz, sz, glm::vec4(0.1f, 1.0f, 0.4f, 0.95f));
                    UI::drawRectOutline(sx - sz * 0.5f, sy - sz * 0.5f, sz, sz, 2.0f, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

                    std::string badge = "#" + std::to_string(i + 1) + ": " + seg.name + (conn.isStart ? " [Start]" : " [End]");
                    UI::drawText(badge, sx + sz * 0.7f, sy - 6.0f * m_uiScale, 0.9f * m_uiScale, glm::vec4(0.2f, 1.0f, 0.5f, 1.0f));
                }
            }
        }

        if (screenEnds.size() >= 2 && validCount >= 2) {
            centroid /= (float)validCount;
            if (!m_isQuadsphereChunk && !m_hasTerrain) {
                centroid.y = 0.05f;
            }
            glm::vec4 clip = vp * glm::vec4(centroid, 1.0f);
            if (clip.w > 0.01f) {
                glm::vec3 ndc = glm::vec3(clip) / clip.w;
                if (ndc.z >= 0.0f && ndc.z <= 1.0f) {
                    float cx = (ndc.x * 0.5f + 0.5f) * (float)screenW;
                    float cy = (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)screenH;

                    // Draw connecting dashed lines from each endpoint to centroid
                    for (const auto& se : screenEnds) {
                        float dist = glm::distance(se, glm::vec2(cx, cy));
                        if (dist > 1.0f) {
                            glm::vec2 dir = (glm::vec2(cx, cy) - se) / dist;
                            for (float d = 0.0f; d < dist; d += 16.0f) {
                                glm::vec2 a = se + dir * d;
                                glm::vec2 b = se + dir * std::min(d + 8.0f, dist);
                                UI::drawRect(a.x - 1.0f, a.y - 1.0f,
                                             std::max(2.0f, std::abs(b.x - a.x) + 2.0f),
                                             std::max(2.0f, std::abs(b.y - a.y) + 2.0f),
                                             glm::vec4(1.0f, 0.85f, 0.1f, 0.85f));
                            }
                        }
                    }

                    // Centroid marker
                    float csz = 16.0f * m_uiScale;
                    UI::drawRect(cx - csz * 0.5f, cy - csz * 0.5f, csz, csz, glm::vec4(1.0f, 0.85f, 0.1f, 1.0f));
                    UI::drawRectOutline(cx - csz * 0.5f, cy - csz * 0.5f, csz, csz, 2.0f, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
                    UI::drawText("Target Junction Center", cx + csz * 0.7f, cy - 6.0f * m_uiScale, 0.95f * m_uiScale, glm::vec4(1.0f, 0.9f, 0.2f, 1.0f));
                }
            }
        }
        return;
    }

    glm::mat4 view = getViewMatrix();
    glm::mat4 proj = getProjMatrix((float)screenW / (float)screenH);
    glm::mat4 vp = proj * view;

    // Loop over all segments in the active chunk
    for (size_t s = 0; s < m_segments.size(); ++s) {
        const auto& seg = m_segments[s];
        if (seg.controlPoints.empty()) continue;

        bool isSelectedSeg = ((int)s == m_selectedSegmentIdx);

        std::vector<glm::vec2> screenPts;
        screenPts.reserve(seg.controlPoints.size());

        for (size_t i = 0; i < seg.controlPoints.size(); ++i) {
            glm::vec4 cpWorld = m_isQuadsphereChunk ? (m_R_ecef_to_enu * glm::vec4(seg.controlPoints[i], 1.0f))
                                                    : glm::vec4(seg.controlPoints[i], 1.0f);
            glm::vec4 clip = vp * cpWorld;
            if (clip.w > 0.01f) {
                glm::vec3 ndc = glm::vec3(clip) / clip.w;
                if (ndc.z >= 0.0f && ndc.z <= 1.0f) {
                    float sx = (ndc.x * 0.5f + 0.5f) * (float)screenW;
                    float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)screenH;
                    screenPts.push_back(glm::vec2(sx, sy));
                } else {
                    screenPts.push_back(glm::vec2(-1000.0f, -1000.0f));
                }
            } else {
                screenPts.push_back(glm::vec2(-1000.0f, -1000.0f));
            }
        }

        // 1. Draw dashed connecting lines along spline control polygon
        glm::vec4 lineColor = isSelectedSeg ? glm::vec4(0.2f, 0.85f, 1.0f, 0.85f)
                                           : glm::vec4(0.55f, 0.60f, 0.65f, 0.35f);
        float lineWidth = isSelectedSeg ? 2.5f : 1.5f;

        for (size_t i = 0; i + 1 < screenPts.size(); ++i) {
            glm::vec2 p0 = screenPts[i];
            glm::vec2 p1 = screenPts[i + 1];
            if (p0.x < 0.0f || p1.x < 0.0f) continue;
            float dist = glm::length(p1 - p0);
            if (dist > 1.0f) {
                glm::vec2 dir = (p1 - p0) / dist;
                float step = 8.0f;
                for (float d = 0.0f; d < dist; d += step * 2.0f) {
                    glm::vec2 a = p0 + dir * d;
                    glm::vec2 b = p0 + dir * std::min(d + step, dist);
                    UI::drawRect(a.x - lineWidth * 0.5f, a.y - lineWidth * 0.5f,
                                 std::max(lineWidth, std::abs(b.x - a.x) + lineWidth),
                                 std::max(lineWidth, std::abs(b.y - a.y) + lineWidth),
                                 lineColor);
                }
            }
        }

        // 2. Draw control point handles
        for (size_t i = 0; i < screenPts.size(); ++i) {
            glm::vec2 pt = screenPts[i];
            if (pt.x < -100.0f || pt.y < -100.0f || pt.x > (float)screenW + 100.0f || pt.y > (float)screenH + 100.0f) continue;

            bool isDragged = (isSelectedSeg && (int)i == m_draggedControlPointIdx);
            bool isSelectedCP = (isSelectedSeg && (int)i == m_selectedControlPointIdx);
            float d2 = (m_mouseX - pt.x) * (m_mouseX - pt.x) + (m_mouseY - pt.y) * (m_mouseY - pt.y);
            bool isHovered = (d2 <= 14.0f * 14.0f * m_uiScale * m_uiScale);

            bool isStart = (i == 0);
            bool isEnd = (i == seg.controlPoints.size() - 1);
            int juncId = isStart ? seg.startJunctionId : (isEnd ? seg.endJunctionId : -1);
            bool isMultiJunc = (juncId >= 0 && juncId < (int)m_junctions.size() && m_junctions[juncId].connections.size() > 1);

            float size;
            glm::vec4 col;
            if (isSelectedSeg) {
                size = isDragged ? 18.0f : (isHovered ? 16.0f : (isSelectedCP ? 15.0f : (isMultiJunc ? 15.0f : 13.0f)));
                if (isDragged) {
                    col = glm::vec4(0.2f, 1.0f, 0.4f, 1.0f);
                } else if (isHovered) {
                    col = glm::vec4(1.0f, 0.9f, 0.2f, 1.0f);
                } else if (isSelectedCP) {
                    col = glm::vec4(1.0f, 0.5f, 0.1f, 1.0f);
                } else if (isMultiJunc) {
                    col = glm::vec4(0.85f, 0.25f, 0.95f, 0.95f);
                } else {
                    col = glm::vec4(0.1f, 0.7f, 1.0f, 0.9f);
                }
            } else {
                // Unselected segment control points: subtle, smaller handles
                size = isHovered ? 12.0f : (isMultiJunc ? 10.0f : 8.0f);
                if (isHovered) {
                    col = glm::vec4(1.0f, 0.9f, 0.2f, 0.9f);
                } else if (isMultiJunc) {
                    col = glm::vec4(0.7f, 0.3f, 0.8f, 0.7f);
                } else {
                    col = glm::vec4(0.5f, 0.55f, 0.6f, 0.6f);
                }
            }
            size *= m_uiScale;

            UI::drawRect(pt.x - size * 0.5f, pt.y - size * 0.5f, size, size, col);
            UI::drawRectOutline(pt.x - size * 0.5f, pt.y - size * 0.5f, size, size, 1.5f, glm::vec4(0.05f, 0.05f, 0.08f, 0.8f));

            // Numbers are ONLY visible for the selected road segment!
            if (isSelectedSeg) {
                std::string label = std::to_string(i);
                if (isMultiJunc) {
                    label += " [Junc: " + std::to_string(m_junctions[juncId].connections.size()) + "-way]";
                }
                UI::drawText(label, pt.x + size * 0.6f, pt.y - 4.0f * m_uiScale, 0.9f * m_uiScale,
                             isMultiJunc ? glm::vec4(1.0f, 0.8f, 1.0f, 0.95f) : glm::vec4(1.0f, 1.0f, 1.0f, 0.95f));
            }
        }
    }
}

void RoadEditor::renderUI(int screenW, int screenH) {
    // 1. Top Header Bar
    float headerH = 40.0f * m_uiScale;
    UI::panel("", 0.0f, 0.0f, (float)screenW, headerH, glm::vec4(0.14f, 0.15f, 0.18f, 0.98f));
    UI::label("ForceEngine Road Editor [Jughandle 500m Demo]", 14.0f * m_uiScale, 10.0f * m_uiScale, 1.25f * m_uiScale, glm::vec4(0.95f, 0.95f, 0.98f, 1.0f));

    // Mode Buttons
    float btnW = 125.0f * m_uiScale;
    float btnH = 26.0f * m_uiScale;
    float btnSpacing = 8.0f * m_uiScale;
    float btnX = (float)screenW - (btnW * 3.0f + btnSpacing * 2.0f + 16.0f * m_uiScale);

    if (UI::button("Top-Down (2D)", btnX, 7.0f * m_uiScale, btnW, btnH, m_viewMode == EditorViewMode::TopDown2D)) {
        m_viewMode = EditorViewMode::TopDown2D;
    }
    if (UI::button("3D Orbit", btnX + btnW + btnSpacing, 7.0f * m_uiScale, btnW, btnH, m_viewMode == EditorViewMode::Orbit3D)) {
        m_viewMode = EditorViewMode::Orbit3D;
    }
    if (UI::button(m_viewMode == EditorViewMode::DriveTest ? "Exit Drive" : "Drive Test",
                   btnX + (btnW + btnSpacing) * 2.0f, 7.0f * m_uiScale, btnW, btnH, m_viewMode == EditorViewMode::DriveTest)) {
        if (m_viewMode == EditorViewMode::DriveTest) exitDriveMode();
        else enterDriveMode(m_selectedSegmentIdx);
    }

    // 2. Drive Mode HUD Overlay
    if (m_viewMode == EditorViewMode::DriveTest) {
        float hudW = 360.0f * m_uiScale;
        float hudH = 115.0f * m_uiScale;
        float hudX = ((float)screenW - hudW) * 0.5f;
        float hudY = (float)screenH - hudH - 24.0f * m_uiScale;

        UI::panel("PASSABILITY SIMULATOR", hudX, hudY, hudW, hudH);

        std::string spdStr = "SPEED: " + std::to_string((int)m_vehicle.getSpeedMph()) + " MPH";
        UI::label(spdStr, hudX + 16.0f * m_uiScale, hudY + 36.0f * m_uiScale, 1.6f * m_uiScale, glm::vec4(0.2f, 0.9f, 1.0f, 1.0f));

        std::string roadStr = "Road: " + m_vehicle.getCurrentRoadName();
        UI::label(roadStr, hudX + 16.0f * m_uiScale, hudY + 64.0f * m_uiScale, 1.0f * m_uiScale, glm::vec4(0.9f, 0.9f, 0.95f, 1.0f));

        std::string statStr = m_vehicle.isOnRoad() ? "Status: [ PASSABLE ON ROAD ]" : "Status: [ OFF ROAD ]";
        glm::vec4 statCol = m_vehicle.isOnRoad() ? glm::vec4(0.2f, 0.95f, 0.3f, 1.0f) : glm::vec4(1.0f, 0.35f, 0.2f, 1.0f);
        UI::label(statStr, hudX + 16.0f * m_uiScale, hudY + 86.0f * m_uiScale, 1.0f * m_uiScale, statCol);

        return; // Minimal HUD during drive test
    }

    // 3. Left Control Panel (Road Toolbox)
    float pX = 14.0f * m_uiScale;
    float pY = 50.0f * m_uiScale;
    float pW = 290.0f * m_uiScale;
    float pH = (float)screenH - pY - 14.0f * m_uiScale;

    UI::panel("ROAD TOOLBOX", pX, pY, pW, pH);

    float curY = pY + 36.0f * m_uiScale;

    // Chunk Selector & Multi-Chunk Domain Decomposition
    if (m_isQuadsphereChunk) {
        UI::label("-- Chunk (Quadsphere L15) --", pX + 14.0f * m_uiScale, curY, 1.0f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
    } else {
        UI::label("-- Chunk Selector (Cartesian Grid) --", pX + 14.0f * m_uiScale, curY, 1.0f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
    }
    curY += 20.0f * m_uiScale;

    // Active Chunk Display Banner
    std::string activeChunkStr;
    if (m_isQuadsphereChunk) {
        activeChunkStr = "Face " + std::to_string(m_activeFace) + " [" +
                         std::to_string(m_activeChunkX) + ", " +
                         std::to_string(m_activeChunkZ) + "]";
    } else {
        activeChunkStr = "[" + std::to_string(m_activeChunkX) + ", " +
                         std::to_string(m_activeChunkZ) + "]";
    }
    UI::panel("", pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale, 26.0f * m_uiScale, glm::vec4(0.14f, 0.17f, 0.22f, 0.95f));
    UI::label("Active: " + activeChunkStr, pX + 22.0f * m_uiScale, curY + 5.0f * m_uiScale, 0.95f * m_uiScale, glm::vec4(0.25f, 0.85f, 1.0f, 1.0f));
    curY += 30.0f * m_uiScale;

    // System Dialog Chunk Input Button
    if (UI::button("Input Chunk (Dialog)...", pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
        uint8_t face = m_activeFace;
        int x = m_activeChunkX, z = m_activeChunkZ;
        bool isQuad = m_isQuadsphereChunk;
        if (openChunkInputDialog(face, x, z, isQuad)) {
            if (isQuad) {
                loadChunk(face, x, z);
            } else {
                loadChunk(x, z);
            }
        }
    }
    curY += 34.0f * m_uiScale;

    float halfW = (pW - 28.0f * m_uiScale);
    float stepBtnW = 32.0f * m_uiScale;
    float lblW = halfW - stepBtnW * 2.0f - 8.0f * m_uiScale;

    // Chunk X / Tile X Selector
    std::string xName = m_isQuadsphereChunk ? "Tile X: " : "Chunk X: ";
    if (UI::button("-", pX + 14.0f * m_uiScale, curY, stepBtnW, 24.0f * m_uiScale)) {
        m_activeChunkX--;
        if (m_isQuadsphereChunk) loadChunk(m_activeFace, m_activeChunkX, m_activeChunkZ);
        else loadChunk(m_activeChunkX, m_activeChunkZ);
    }
    UI::panel("", pX + 14.0f * m_uiScale + stepBtnW + 4.0f * m_uiScale, curY, lblW, 24.0f * m_uiScale, glm::vec4(0.18f, 0.20f, 0.24f, 0.95f));
    UI::label(xName + std::to_string(m_activeChunkX), pX + 14.0f * m_uiScale + stepBtnW + 12.0f * m_uiScale, curY + 4.0f * m_uiScale, 0.95f * m_uiScale, glm::vec4(0.95f));
    if (UI::button("+", pX + 14.0f * m_uiScale + stepBtnW + lblW + 8.0f * m_uiScale, curY, stepBtnW, 24.0f * m_uiScale)) {
        m_activeChunkX++;
        if (m_isQuadsphereChunk) loadChunk(m_activeFace, m_activeChunkX, m_activeChunkZ);
        else loadChunk(m_activeChunkX, m_activeChunkZ);
    }
    curY += 28.0f * m_uiScale;

    // Chunk Z / Tile Y Selector
    std::string zName = m_isQuadsphereChunk ? "Tile Y: " : "Chunk Z: ";
    if (UI::button("-", pX + 14.0f * m_uiScale, curY, stepBtnW, 24.0f * m_uiScale)) {
        m_activeChunkZ--;
        if (m_isQuadsphereChunk) loadChunk(m_activeFace, m_activeChunkX, m_activeChunkZ);
        else loadChunk(m_activeChunkX, m_activeChunkZ);
    }
    UI::panel("", pX + 14.0f * m_uiScale + stepBtnW + 4.0f * m_uiScale, curY, lblW, 24.0f * m_uiScale, glm::vec4(0.18f, 0.20f, 0.24f, 0.95f));
    UI::label(zName + std::to_string(m_activeChunkZ), pX + 14.0f * m_uiScale + stepBtnW + 12.0f * m_uiScale, curY + 4.0f * m_uiScale, 0.95f * m_uiScale, glm::vec4(0.95f));
    if (UI::button("+", pX + 14.0f * m_uiScale + stepBtnW + lblW + 8.0f * m_uiScale, curY, stepBtnW, 24.0f * m_uiScale)) {
        m_activeChunkZ++;
        if (m_isQuadsphereChunk) loadChunk(m_activeFace, m_activeChunkX, m_activeChunkZ);
        else loadChunk(m_activeChunkX, m_activeChunkZ);
    }
    curY += 32.0f * m_uiScale;

    if (UI::button("Reload Chunk", pX + 14.0f * m_uiScale, curY, halfW * 0.48f, 26.0f * m_uiScale)) {
        if (m_isQuadsphereChunk) loadChunk(m_activeFace, m_activeChunkX, m_activeChunkZ);
        else loadChunk(m_activeChunkX, m_activeChunkZ);
    }
    if (UI::button("Save (.froad)", pX + 14.0f * m_uiScale + halfW * 0.52f, curY, halfW * 0.48f, 26.0f * m_uiScale)) {
        saveActiveChunk();
    }
    curY += 34.0f * m_uiScale;

    std::string chunkStatus = "Adjacent: " + std::to_string(m_adjacentMeshes.size()) + " visible";
    UI::label(chunkStatus, pX + 14.0f * m_uiScale, curY, 0.85f * m_uiScale, glm::vec4(0.6f, 0.8f, 0.95f, 0.9f));
    curY += 18.0f * m_uiScale;

    // GPS Telemetry
    double chunkLat = 0.0, chunkLon = 0.0;
    if (m_isQuadsphereChunk) {
        if (glm::length(m_chunkCenterECEF) > 1000.0) {
            double dummyAlt;
            Quadsphere::ecefToGeodetic(m_chunkCenterECEF, chunkLat, chunkLon, dummyAlt);
        } else {
            double minU, maxU, minV, maxV;
            Quadsphere::getTileUVBounds(m_activeLOD, m_activeChunkX, m_activeChunkZ, minU, maxU, minV, maxV);
            glm::dvec3 centerECEF = Quadsphere::quadsphereToECEF(m_activeFace, (minU + maxU) * 0.5, (minV + maxV) * 0.5, 0.0);
            double dummyAlt;
            Quadsphere::ecefToGeodetic(centerECEF, chunkLat, chunkLon, dummyAlt);
        }
    } else {
        const double cLat = 40.592676;
        const double cLon = -74.515211;
        const double degToRad = 3.14159265358979323846 / 180.0;
        const double m_per_lat = 111139.0;
        const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);
        chunkLat = cLat - (m_activeChunkZ * 512.0 / m_per_lat);
        chunkLon = cLon + (m_activeChunkX * 512.0 / m_per_lon);
    }

    std::stringstream ssGps;
    ssGps << std::fixed << std::setprecision(5) << chunkLat << " N, " << std::abs(chunkLon) << " W";
    UI::label("GPS: " + ssGps.str(), pX + 14.0f * m_uiScale, curY, 0.82f * m_uiScale, glm::vec4(0.4f, 0.9f, 0.5f, 0.95f));
    curY += 18.0f * m_uiScale;

    std::string terrainType = m_isQuadsphere ? "Terrain: Quadsphere DEM" : (m_hasTerrain ? "Terrain: Planar DEM" : "Terrain: None");
    UI::label(terrainType, pX + 14.0f * m_uiScale, curY, 0.82f * m_uiScale, glm::vec4(0.85f, 0.85f, 0.4f, 0.95f));
    curY += 20.0f * m_uiScale;

    UI::separator(pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale);
    curY += 12.0f * m_uiScale;

    // Standards & Global Overrides
    UI::label("-- Highway Standards --", pX + 14.0f * m_uiScale, curY, 1.0f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
    curY += 20.0f * m_uiScale;

    if (UI::button("Apply AASHTO Standards", pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale, 26.0f * m_uiScale)) {
        rebuildRoadGeometry();
    }
    curY += 32.0f * m_uiScale;

    UI::separator(pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale);
    curY += 12.0f * m_uiScale;

    // Topology Tools
    UI::label("-- Topology Tools --", pX + 14.0f * m_uiScale, curY, 1.0f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
    curY += 20.0f * m_uiScale;

    std::string juncBtn = m_isAddingJunction ? "Cancel Add Junction [Esc]" : "+ Add Junction Tool";
    if (UI::button(juncBtn, pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale, 28.0f * m_uiScale, m_isAddingJunction)) {
        m_isAddingJunction = !m_isAddingJunction;
        if (!m_isAddingJunction) {
            m_pendingJunctionConns.clear();
        } else {
            m_selectedSegmentIdx = -1;
            m_selectedMesh = nullptr;
            m_selectedControlPointIdx = -1;
            m_draggedControlPointIdx = -1;
            m_isDraggingControlPoint = false;
            m_statusMessage = "Click road endpoints or road bodies to select, then click 'Create Junction'.";
            m_statusTimer = 6.0f;
        }
    }
    curY += 34.0f * m_uiScale;

    std::string viewJuncsBtn = "Junction Inspector (" + std::to_string(m_junctions.size()) + ") ->";
    if (UI::button(viewJuncsBtn, pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale, 26.0f * m_uiScale)) {
        m_rightPanelTab = 2;
    }
    curY += 32.0f * m_uiScale;

    UI::separator(pX + 14.0f * m_uiScale, curY, pW - 28.0f * m_uiScale);
    curY += 12.0f * m_uiScale;

    // Statistics
    UI::label("-- Network Statistics --", pX + 14.0f * m_uiScale, curY, 1.0f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
    curY += 18.0f * m_uiScale;

    int totalSegs = (int)m_segments.size();
    int in500Count = (int)std::count_if(m_segments.begin(), m_segments.end(),
                                        [](const ProcessedRoadSegment& s){ return s.in500mChunk; });

    UI::label("Total Segments: " + std::to_string(totalSegs), pX + 14.0f * m_uiScale, curY, 0.95f * m_uiScale);
    curY += 16.0f * m_uiScale;
    UI::label("In Active Chunk: " + std::to_string(in500Count), pX + 14.0f * m_uiScale, curY, 0.95f * m_uiScale);
    curY += 16.0f * m_uiScale;
    if (m_mergedMesh) {
        UI::label("Vertices: " + std::to_string(m_mergedMesh->vertexCount), pX + 14.0f * m_uiScale, curY, 0.95f * m_uiScale);
        curY += 16.0f * m_uiScale;
        UI::label("Triangles: " + std::to_string(m_mergedMesh->indexCount / 3), pX + 14.0f * m_uiScale, curY, 0.95f * m_uiScale);
        curY += 16.0f * m_uiScale;
    }

    // 4. Right Panel: Unified Elevation Solver, Road Inspector, & Junction Inspector
    float inspW = 320.0f * m_uiScale;
    float inspY = 50.0f * m_uiScale;
    float maxInspH = std::max(200.0f * m_uiScale, (float)screenH - inspY - 14.0f * m_uiScale);
    float inspH = std::min(m_rightPanelHeight, maxInspH);
    float inspX = (float)screenW - inspW - 14.0f * m_uiScale;

    std::string panelHeader = "INSPECTOR";
    if (m_rightPanelTab == 0) panelHeader = "ELEVATION SOLVER";
    else if (m_rightPanelTab == 1) panelHeader = "ROAD INSPECTOR";
    else if (m_rightPanelTab == 2) panelHeader = "JUNCTION INSPECTOR";

    UI::panel(panelHeader, inspX, inspY, inspW, inspH);

    float iy = inspY + 36.0f * m_uiScale;

    // Reset scroll when changing segment, junction, or active tab
    static int s_lastSegmentIdx = -1;
    static int s_lastJuncIdx = -1;
    static int s_lastTab = -1;
    if (m_selectedSegmentIdx != s_lastSegmentIdx || m_selectedJunctionIdx != s_lastJuncIdx || m_rightPanelTab != s_lastTab) {
        m_rightPanelScrollY = 0.0f;
        s_lastSegmentIdx = m_selectedSegmentIdx;
        s_lastJuncIdx = m_selectedJunctionIdx;
        s_lastTab = m_rightPanelTab;
    }

    // 3 Tab Selector Buttons: [Solver] [Road *] [Junctions *]
    float tabSpacing = 4.0f * m_uiScale;
    float tabW = (inspW - 28.0f * m_uiScale - tabSpacing * 2.0f) / 3.0f;
    float tabH = 26.0f * m_uiScale;

    if (UI::button("Solver", inspX + 14.0f * m_uiScale, iy, tabW, tabH, m_rightPanelTab == 0)) {
        m_rightPanelTab = 0;
        m_rightPanelScrollY = 0.0f;
    }
    std::string roadTabTitle = (m_selectedSegmentIdx >= 0) ? "Road *" : "Road";
    if (UI::button(roadTabTitle, inspX + 14.0f * m_uiScale + tabW + tabSpacing, iy, tabW, tabH, m_rightPanelTab == 1)) {
        m_rightPanelTab = 1;
        m_rightPanelScrollY = 0.0f;
    }
    std::string juncTabTitle = (m_selectedJunctionIdx >= 0) ? "Junc *" : "Junctions";
    if (UI::button(juncTabTitle, inspX + 14.0f * m_uiScale + (tabW + tabSpacing) * 2.0f, iy, tabW, tabH, m_rightPanelTab == 2)) {
        m_rightPanelTab = 2;
        m_rightPanelScrollY = 0.0f;
    }
    iy += 34.0f * m_uiScale;

    float contentStartY = iy;
    float clipBottom = inspY + inspH - 6.0f * m_uiScale;

    // Mouse wheel scrolling when hovering over right panel
    bool mouseOverRightPanel = (m_mouseX >= inspX && m_mouseX <= inspX + inspW &&
                                m_mouseY >= inspY && m_mouseY <= inspY + inspH);
    if (mouseOverRightPanel && std::abs(m_mouseWheel) > 0.01f) {
        m_rightPanelScrollY -= m_mouseWheel * 32.0f;
    }
    float maxScroll = std::max(0.0f, m_rightPanelHeight - maxInspH);
    m_rightPanelScrollY = std::clamp(m_rightPanelScrollY, 0.0f, maxScroll);

    // Apply scroll offset to content
    iy -= m_rightPanelScrollY;

    auto canDraw = [&](float elemY, float elemH) {
        return (elemY + elemH >= contentStartY - 2.0f && elemY <= clipBottom + 2.0f);
    };

        if (m_rightPanelTab == 0) {
            // === TAB 0: ELEVATION SOLVER TUNING PANEL ===
            bool optionsChanged = false;

            std::string statusText;
            if (m_hasTerrain) {
                statusText = "Status: Converged (" + std::to_string((int)m_lastSolveTimeMs) + "ms, " +
                             std::to_string(m_lastSolveSweeps) + " sweeps)";
            } else {
                statusText = "Status: No Terrain Loaded";
            }
            UI::label(statusText, inspX + 14.0f * m_uiScale, iy, 0.90f * m_uiScale, glm::vec4(0.2f, 0.95f, 0.4f, 1.0f));
            iy += 20.0f * m_uiScale;

            UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
            iy += 10.0f * m_uiScale;

            // 1. Competing Physical Forces
            UI::label("-- Physical Forces --", inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.7f, 0.85f, 1.0f, 1.0f));
            iy += 20.0f * m_uiScale;

            if (UI::sliderFloat("Tension (kt)", m_solverOptions.k_tension, 0.0f, 400.0f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            if (UI::sliderFloat("Adhesion (k_adh)", m_solverOptions.k_adhesion, 0.0f, 50.0f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            if (UI::sliderFloat("Repulsion (k_rep)", m_solverOptions.k_repulsion, 0.0f, 500.0f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
            iy += 10.0f * m_uiScale;

            // 2. Clearances & Geometric Offsets
            UI::label("-- Constraints & Clearance --", inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.7f, 0.85f, 1.0f, 1.0f));
            iy += 20.0f * m_uiScale;

            if (UI::sliderFloat("Min Clearance", m_solverOptions.minClearance, 2.5f, 10.0f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            if (UI::sliderFloat("Ground Offset", m_solverOptions.groundOffset, 0.05f, 0.50f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            if (UI::sliderFloat("Max Subdiv Span", m_solverOptions.maxSegmentLength, 6.0f, 25.0f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
            iy += 10.0f * m_uiScale;

            // 3. Gauss-Seidel Relaxation Parameters
            UI::label("-- Solver Relaxation --", inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.7f, 0.85f, 1.0f, 1.0f));
            iy += 20.0f * m_uiScale;

            if (UI::sliderFloat("SOR Omega (w)", m_solverOptions.omega, 1.00f, 1.60f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            float sweepsFloat = (float)m_solverOptions.maxSweeps;
            if (UI::sliderFloat("Max Sweeps", sweepsFloat, 20.0f, 300.0f,
                                inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                m_solverOptions.maxSweeps = (int)sweepsFloat;
                optionsChanged = true;
            }
            iy += 36.0f * m_uiScale;

            UI::toggle("Live Auto-Solve", m_autoSolveElevation, inspX + 14.0f * m_uiScale, iy);
            iy += 32.0f * m_uiScale;

            float actBtnW = (inspW - 28.0f * m_uiScale - 8.0f * m_uiScale) * 0.5f;
            if (UI::button("Solve Now", inspX + 14.0f * m_uiScale, iy, actBtnW, 26.0f * m_uiScale)) {
                solveRoadElevations();
            }
            if (UI::button("Reset Defaults", inspX + 14.0f * m_uiScale + actBtnW + 8.0f * m_uiScale, iy, actBtnW, 26.0f * m_uiScale)) {
                m_solverOptions = RoadElevationSolverOptions();
                solveRoadElevations();
            }

            // Immediately visible adjustments:
            if (optionsChanged && m_autoSolveElevation) {
                solveRoadElevations();
            }
        } else if (m_rightPanelTab == 1) {
            // === TAB 1: ROAD INSPECTOR ===
            if (m_selectedSegmentIdx >= 0 && m_selectedSegmentIdx < (int)m_segments.size()) {
                auto& seg = m_segments[m_selectedSegmentIdx];

                UI::label("Road: " + seg.name, inspX + 14.0f * m_uiScale, iy, 1.1f * m_uiScale, glm::vec4(1.0f, 0.9f, 0.3f, 1.0f));
                iy += 20.0f * m_uiScale;

                std::string classStr = "Class: " + seg.highway;
                if (!seg.ref.empty()) classStr += " (" + seg.ref + ")";
                UI::label(classStr, inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.7f, 0.85f, 1.0f, 1.0f));
                iy += 18.0f * m_uiScale;

                if (UI::button("Delete Road Segment [Del]", inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 26.0f * m_uiScale)) {
                    deleteSegment(m_selectedSegmentIdx);
                    return;
                }
                iy += 30.0f * m_uiScale;

                UI::label(seg.widthSource, inspX + 14.0f * m_uiScale, iy, 0.85f * m_uiScale, glm::vec4(0.55f, 0.95f, 0.65f, 1.0f));
                iy += 22.0f * m_uiScale;

                UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                iy += 12.0f * m_uiScale;

                bool segChanged = false;

                // Editable Lanes count with [-] and [+] buttons
                UI::label("Lanes: " + std::to_string(seg.lanes), inspX + 14.0f * m_uiScale, iy + 4.0f * m_uiScale, 1.0f * m_uiScale);
                if (UI::button("-", inspX + 110.0f * m_uiScale, iy, 32.0f * m_uiScale, 24.0f * m_uiScale)) {
                    if (seg.lanes > 1) { seg.lanes--; seg.widthSource = "User Custom"; segChanged = true; }
                }
                if (UI::button("+", inspX + 148.0f * m_uiScale, iy, 32.0f * m_uiScale, 24.0f * m_uiScale)) {
                    if (seg.lanes < 8) { seg.lanes++; seg.widthSource = "User Custom"; segChanged = true; }
                }
                iy += 32.0f * m_uiScale;

                // One-Way Toggle
                if (UI::toggle("One-Way Road", seg.oneway, inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 24.0f * m_uiScale)) {
                    seg.widthSource = "User Custom";
                    segChanged = true;
                }
                iy += 32.0f * m_uiScale;

                // Lane Width Slider
                if (UI::sliderFloat("Lane Width", seg.laneWidth, 2.50f, 5.00f,
                                    inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                    seg.widthSource = "User Custom";
                    segChanged = true;
                }
                iy += 36.0f * m_uiScale;

                // Shoulder Width Slider
                if (UI::sliderFloat("Shoulder", seg.shoulderWidth, 0.00f, 4.00f,
                                    inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                    seg.widthSource = "User Custom";
                    segChanged = true;
                }
                iy += 36.0f * m_uiScale;

                // Curve Sampling Step
                if (UI::sliderFloat("Curve Step", seg.sampleStep, 1.00f, 5.00f,
                                    inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                    segChanged = true;
                }
                iy += 34.0f * m_uiScale;

                // Vertical Elevation & Layer Controls
                std::string layerStr = "Layer: " + std::to_string(seg.layer);
                if (seg.isBridge) layerStr += " [Bridge Overpass]";
                else if (seg.isTunnel) layerStr += " [Tunnel]";
                else if (seg.elevation > 1.0f) layerStr += " [Elevated]";
                UI::label(layerStr, inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.85f, 0.85f, 1.0f, 1.0f));
                iy += 20.0f * m_uiScale;

                float halfLayerBtn = (inspW - 36.0f * m_uiScale) * 0.5f;
                if (UI::button("Lower Layer [-]", inspX + 14.0f * m_uiScale, iy, halfLayerBtn, 24.0f * m_uiScale)) {
                    seg.layer--;
                    float diff = (float)seg.layer * 5.5f + 0.05f - seg.elevation;
                    seg.elevation += diff;
                    for (auto& cp : seg.controlPoints) cp.y += diff;
                    seg.isBridge = (seg.layer > 0);
                    segChanged = true;
                }
                if (UI::button("Raise Layer [+]", inspX + 22.0f * m_uiScale + halfLayerBtn, iy, halfLayerBtn, 24.0f * m_uiScale)) {
                    seg.layer++;
                    float diff = (float)seg.layer * 5.5f + 0.05f - seg.elevation;
                    seg.elevation += diff;
                    for (auto& cp : seg.controlPoints) cp.y += diff;
                    seg.isBridge = (seg.layer > 0);
                    segChanged = true;
                }
                iy += 28.0f * m_uiScale;

                if (UI::sliderFloat("Height (m)", seg.elevation, -5.0f, 15.0f,
                                    inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 26.0f * m_uiScale)) {
                    for (auto& cp : seg.controlPoints) cp.y = seg.elevation;
                    seg.isBridge = (seg.elevation >= 2.0f);
                    segChanged = true;
                }
                iy += 32.0f * m_uiScale;

                UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                iy += 10.0f * m_uiScale;

                // Control Points Dragging & Management Info
                std::string cpInfo = "Control Points: " + std::to_string(seg.controlPoints.size());
                if (m_selectedControlPointIdx >= 0 && m_selectedControlPointIdx < (int)seg.controlPoints.size()) {
                    cpInfo += " [#" + std::to_string(m_selectedControlPointIdx) + " Selected]";
                }
                UI::label(cpInfo, inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.9f, 0.9f, 0.95f, 1.0f));
                iy += 22.0f * m_uiScale;

                float halfBtnW = (inspW - 36.0f * m_uiScale) * 0.5f;
                if (UI::button("Insert Point Before", inspX + 14.0f * m_uiScale, iy, halfBtnW, 26.0f * m_uiScale)) {
                    int idx = (m_selectedControlPointIdx >= 0) ? m_selectedControlPointIdx : 0;
                    if (idx <= 0 && !seg.controlPoints.empty()) {
                        // Extend road backwards from first control point
                        glm::vec3 dir(0.0f, 0.0f, -1.0f);
                        if (seg.controlPoints.size() >= 2) {
                            dir = glm::normalize(seg.controlPoints[0] - seg.controlPoints[1]);
                        }
                        glm::vec3 newPt = seg.controlPoints.front() + dir * 15.0f;
                        seg.controlPoints.insert(seg.controlPoints.begin(), newPt);
                        seg.startJunctionId = -1; // Detached extension
                        m_selectedControlPointIdx = 0;
                        segChanged = true;
                    } else if (idx > 0 && idx < (int)seg.controlPoints.size()) {
                        glm::vec3 mid = (seg.controlPoints[idx - 1] + seg.controlPoints[idx]) * 0.5f;
                        seg.controlPoints.insert(seg.controlPoints.begin() + idx, mid);
                        m_selectedControlPointIdx = idx;
                        segChanged = true;
                    }
                }
                if (UI::button("Insert Point After", inspX + 18.0f * m_uiScale + halfBtnW, iy, halfBtnW, 26.0f * m_uiScale)) {
                    int idx = (m_selectedControlPointIdx >= 0) ? m_selectedControlPointIdx : ((int)seg.controlPoints.size() - 1);
                    int n = (int)seg.controlPoints.size();
                    if (idx >= n - 1 && n > 0) {
                        // Extend road forward from last control point
                        glm::vec3 dir(0.0f, 0.0f, 1.0f);
                        if (n >= 2) {
                            dir = glm::normalize(seg.controlPoints[n - 1] - seg.controlPoints[n - 2]);
                        }
                        glm::vec3 newPt = seg.controlPoints.back() + dir * 15.0f;
                        seg.controlPoints.push_back(newPt);
                        seg.endJunctionId = -1; // Detached extension
                        m_selectedControlPointIdx = (int)seg.controlPoints.size() - 1;
                        segChanged = true;
                    } else if (idx >= 0 && idx < n - 1) {
                        glm::vec3 mid = (seg.controlPoints[idx] + seg.controlPoints[idx + 1]) * 0.5f;
                        seg.controlPoints.insert(seg.controlPoints.begin() + idx + 1, mid);
                        m_selectedControlPointIdx = idx + 1;
                        segChanged = true;
                    }
                }
                iy += 30.0f * m_uiScale;

                if (UI::button("Delete Point", inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 24.0f * m_uiScale)) {
                    if (seg.controlPoints.size() > 2) {
                        int delIdx = (m_selectedControlPointIdx >= 0) ? m_selectedControlPointIdx : (int)seg.controlPoints.size() - 1;
                        seg.controlPoints.erase(seg.controlPoints.begin() + delIdx);
                        m_selectedControlPointIdx = std::min(delIdx, (int)seg.controlPoints.size() - 1);
                        segChanged = true;
                    }
                }
                iy += 30.0f * m_uiScale;

                // Split Segment at Selected Control Point
                bool canSplit = (m_selectedControlPointIdx > 0 && m_selectedControlPointIdx < (int)seg.controlPoints.size() - 1);
                std::string splitLabel = canSplit ? ("Split at CP #" + std::to_string(m_selectedControlPointIdx)) : "Split (Select Interior CP)";
                if (UI::button(splitLabel, inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 26.0f * m_uiScale, canSplit)) {
                    if (canSplit) {
                        splitSegmentAtControlPoint(m_selectedSegmentIdx, m_selectedControlPointIdx);
                        return;
                    } else {
                        m_statusMessage = "Select an interior control point (between start and end) to split this road.";
                        m_statusTimer = 3.5f;
                    }
                }
                iy += 32.0f * m_uiScale;

                // Individual Control Point Height Tuning
                if (m_selectedControlPointIdx >= 0 && m_selectedControlPointIdx < (int)seg.controlPoints.size()) {
                    float& cpY = seg.controlPoints[m_selectedControlPointIdx].y;
                    if (UI::sliderFloat("CP Height", cpY, -5.0f, 15.0f,
                                        inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 26.0f * m_uiScale)) {
                        segChanged = true;
                    }
                    iy += 32.0f * m_uiScale;
                }

                UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                iy += 10.0f * m_uiScale;

                // Network Topology Connectivity
                UI::label("-- Network Topology --", inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
                iy += 18.0f * m_uiScale;

                auto showJunctionSection = [&](const char* tag, int juncId, bool isStart, float& curY) {
                    if (juncId >= 0 && juncId < (int)m_junctions.size()) {
                        const auto& junc = m_junctions[juncId];
                        std::string header = std::string(tag) + " Junc #" + std::to_string(junc.id) +
                                             " (" + std::to_string(junc.connections.size()) + "-way)";
                        if (canDraw(curY, 18.0f * m_uiScale)) {
                            UI::label(header, inspX + 14.0f * m_uiScale, curY, 0.88f * m_uiScale, glm::vec4(0.85f, 0.5f, 1.0f, 1.0f));
                        }
                        curY += 18.0f * m_uiScale;

                        float juncBtnW = (inspW - 36.0f * m_uiScale) * 0.5f;
                        std::string inspJuncBtn = "Inspect Junc #" + std::to_string(junc.id);
                        if (canDraw(curY, 22.0f * m_uiScale)) {
                            if (UI::button(inspJuncBtn, inspX + 14.0f * m_uiScale, curY, juncBtnW, 22.0f * m_uiScale)) {
                                m_selectedJunctionIdx = juncId;
                                m_rightPanelTab = 2;
                                return;
                            }
                        }
                        std::string remJuncBtn = "Remove Junc";
                        if (canDraw(curY, 22.0f * m_uiScale)) {
                            if (UI::button(remJuncBtn, inspX + 22.0f * m_uiScale + juncBtnW, curY, juncBtnW, 22.0f * m_uiScale)) {
                                removeJunction(juncId);
                                return;
                            }
                        }
                        curY += 26.0f * m_uiScale;

                        std::string discBtn = "Disconnect " + std::string(tag) + " Junction";
                        if (canDraw(curY, 22.0f * m_uiScale)) {
                            if (UI::button(discBtn, inspX + 14.0f * m_uiScale, curY, inspW - 28.0f * m_uiScale, 22.0f * m_uiScale)) {
                                disconnectSegmentFromJunction(m_selectedSegmentIdx, isStart);
                                return;
                            }
                        }
                        curY += 26.0f * m_uiScale;

                        for (const auto& conn : junc.connections) {
                            if (conn.segmentIdx >= 0 && conn.segmentIdx < (int)m_segments.size()) {
                                if (conn.segmentIdx != m_selectedSegmentIdx) {
                                    std::string label = "-> " + m_segments[conn.segmentIdx].name;
                                    if (label.length() > 28) label = label.substr(0, 25) + "...";
                                    if (canDraw(curY, 20.0f * m_uiScale)) {
                                        if (UI::button(label, inspX + 24.0f * m_uiScale, curY, inspW - 38.0f * m_uiScale, 20.0f * m_uiScale)) {
                                            m_selectedSegmentIdx = conn.segmentIdx;
                                            m_selectedMesh = m_segments[conn.segmentIdx].visualMesh;
                                            m_selectedControlPointIdx = -1;
                                        }
                                    }
                                    curY += 23.0f * m_uiScale;
                                }
                            }
                        }
                    } else {
                        std::string header = std::string(tag) + " Junction: Dead End";
                        if (canDraw(curY, 16.0f * m_uiScale)) {
                            UI::label(header, inspX + 14.0f * m_uiScale, curY, 0.85f * m_uiScale, glm::vec4(0.6f, 0.6f, 0.7f, 1.0f));
                        }
                        curY += 16.0f * m_uiScale;
                    }
                };

                showJunctionSection("Start", seg.startJunctionId, true, iy);
                iy += 4.0f * m_uiScale;
                showJunctionSection("End", seg.endJunctionId, false, iy);
                iy += 10.0f * m_uiScale;

                if (canDraw(iy, 2.0f)) {
                    UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                }
                iy += 10.0f * m_uiScale;

                if (canDraw(iy, 26.0f * m_uiScale)) {
                    if (UI::button("Delete Road Segment [Del]", inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 26.0f * m_uiScale)) {
                        deleteSegment(m_selectedSegmentIdx);
                        return;
                    }
                }
                iy += 30.0f * m_uiScale;

                if (segChanged) {
                    rebuildSegmentGeometry(m_selectedSegmentIdx);
                }
            } else {
                UI::label("No road segment selected.", inspX + 14.0f * m_uiScale, iy, 1.0f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
                iy += 24.0f * m_uiScale;
                UI::label("Click on any road in the 2D or 3D view", inspX + 14.0f * m_uiScale, iy, 0.88f * m_uiScale, glm::vec4(0.55f, 0.6f, 0.7f, 1.0f));
                iy += 18.0f * m_uiScale;
                UI::label("to inspect its lane configuration, profile,", inspX + 14.0f * m_uiScale, iy, 0.88f * m_uiScale, glm::vec4(0.55f, 0.6f, 0.7f, 1.0f));
                iy += 18.0f * m_uiScale;
                UI::label("elevations, and junction topology.", inspX + 14.0f * m_uiScale, iy, 0.88f * m_uiScale, glm::vec4(0.55f, 0.6f, 0.7f, 1.0f));
                iy += 28.0f * m_uiScale;

                UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                iy += 14.0f * m_uiScale;

                if (UI::button("Switch to Elevation Solver", inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                    m_rightPanelTab = 0;
                }
            }
        } else if (m_rightPanelTab == 2) {
            // === TAB 2: JUNCTION INSPECTOR ===
            if (m_selectedJunctionIdx >= 0 && m_selectedJunctionIdx < (int)m_junctions.size()) {
                const auto& junc = m_junctions[m_selectedJunctionIdx];
                std::string title = "Junction #" + std::to_string(junc.id) + " (" + std::to_string(junc.connections.size()) + "-way)";
                UI::label(title, inspX + 14.0f * m_uiScale, iy, 1.1f * m_uiScale, glm::vec4(1.0f, 0.5f, 0.95f, 1.0f));
                iy += 22.0f * m_uiScale;

                std::string posStr = "Center: [" + std::to_string((int)junc.position.x) + ", " +
                                     std::to_string((int)junc.position.y) + ", " +
                                     std::to_string((int)junc.position.z) + "]";
                UI::label(posStr, inspX + 14.0f * m_uiScale, iy, 0.88f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
                iy += 18.0f * m_uiScale;

                std::string elevStr = "Elevation: " + std::to_string(junc.position.y).substr(0, 5) + " m";
                UI::label(elevStr, inspX + 14.0f * m_uiScale, iy, 0.88f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
                iy += 22.0f * m_uiScale;

                UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                iy += 12.0f * m_uiScale;

                // Prominent Remove Junction button
                if (UI::button("Remove Junction [Del]", inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 28.0f * m_uiScale)) {
                    removeJunction(m_selectedJunctionIdx);
                    return;
                }
                iy += 34.0f * m_uiScale;

                UI::label("-- Connected Roads --", inspX + 14.0f * m_uiScale, iy, 0.95f * m_uiScale, glm::vec4(0.7f, 0.85f, 1.0f, 1.0f));
                iy += 20.0f * m_uiScale;

                for (size_t c = 0; c < junc.connections.size(); ++c) {
                    const auto& conn = junc.connections[c];
                    if (conn.segmentIdx >= 0 && conn.segmentIdx < (int)m_segments.size()) {
                        const auto& cSeg = m_segments[conn.segmentIdx];
                        std::string roadName = cSeg.name;
                        if (roadName.empty()) roadName = "Seg #" + std::to_string(conn.segmentIdx);
                        if (roadName.length() > 20) roadName = roadName.substr(0, 18) + "..";

                        std::string info = roadName + (conn.isStart ? " [Start]" : " [End]");
                        UI::label(info, inspX + 18.0f * m_uiScale, iy + 4.0f * m_uiScale, 0.88f * m_uiScale, glm::vec4(0.9f, 0.9f, 0.95f, 1.0f));

                        float btnX = inspX + inspW - 130.0f * m_uiScale;
                        if (UI::button("Select", btnX, iy, 56.0f * m_uiScale, 22.0f * m_uiScale)) {
                            m_selectedSegmentIdx = conn.segmentIdx;
                            m_selectedMesh = m_segments[conn.segmentIdx].visualMesh;
                            m_selectedControlPointIdx = -1;
                            m_rightPanelTab = 1;
                            return;
                        }
                        if (UI::button("Disc", btnX + 60.0f * m_uiScale, iy, 52.0f * m_uiScale, 22.0f * m_uiScale)) {
                            disconnectSegmentFromJunction(conn.segmentIdx, conn.isStart);
                            return;
                        }
                        iy += 26.0f * m_uiScale;
                    }
                }

                iy += 10.0f * m_uiScale;
                UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                iy += 14.0f * m_uiScale;

                if (UI::button("Deselect Junction", inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 26.0f * m_uiScale)) {
                    m_selectedJunctionIdx = -1;
                }
            } else {
                std::string header = "Active Chunk Junctions (" + std::to_string(m_junctions.size()) + ")";
                UI::label(header, inspX + 14.0f * m_uiScale, iy, 1.05f * m_uiScale, glm::vec4(1.0f, 0.8f, 0.3f, 1.0f));
                iy += 22.0f * m_uiScale;

                UI::label("Click a junction to inspect or remove it:", inspX + 14.0f * m_uiScale, iy, 0.85f * m_uiScale, glm::vec4(0.7f, 0.75f, 0.85f, 1.0f));
                iy += 20.0f * m_uiScale;

                UI::separator(inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale);
                iy += 10.0f * m_uiScale;

                if (m_junctions.empty()) {
                    UI::label("No junctions found in this chunk.", inspX + 14.0f * m_uiScale, iy, 0.90f * m_uiScale, glm::vec4(0.6f, 0.65f, 0.75f, 1.0f));
                    iy += 20.0f * m_uiScale;
                } else {
                    int displayed = 0;
                    for (int j = 0; j < (int)m_junctions.size(); ++j) {
                        const auto& junc = m_junctions[j];
                        std::string jName = "Junction #" + std::to_string(junc.id) + " (" +
                                            std::to_string(junc.connections.size()) + "-way)";

                        if (UI::button(jName, inspX + 14.0f * m_uiScale, iy, inspW - 28.0f * m_uiScale, 24.0f * m_uiScale)) {
                            m_selectedJunctionIdx = j;
                            m_cameraFocus = junc.position;
                        }
                        iy += 28.0f * m_uiScale;
                        displayed++;
                        if (displayed >= 18) {
                            UI::label("... (" + std::to_string(m_junctions.size() - displayed) + " more)",
                                      inspX + 14.0f * m_uiScale, iy, 0.82f * m_uiScale, glm::vec4(0.6f, 0.6f, 0.7f, 1.0f));
                            break;
                        }
                    }
                }
            }
        }

    // Dynamic height calculation & scrollbar rendering for Right Panel
    float totalContentY = iy + m_rightPanelScrollY;
    m_rightPanelHeight = std::max(180.0f * m_uiScale, totalContentY - inspY + 16.0f * m_uiScale);

    if (maxScroll > 0.0f && inspH >= 120.0f * m_uiScale) {
        float trackX = inspX + inspW - 8.0f * m_uiScale;
        float trackY = contentStartY;
        float trackH = inspH - (contentStartY - inspY) - 6.0f * m_uiScale;
        float thumbH = std::max(24.0f * m_uiScale, trackH * (maxInspH / m_rightPanelHeight));
        float thumbY = trackY + (m_rightPanelScrollY / maxScroll) * (trackH - thumbH);
        UI::drawRect(trackX, trackY, 4.0f * m_uiScale, trackH, glm::vec4(0.18f, 0.20f, 0.25f, 0.6f));
        UI::drawRect(trackX, thumbY, 4.0f * m_uiScale, thumbH, glm::vec4(0.45f, 0.50f, 0.60f, 0.85f));
    }

    // 5. Floating Junction Builder Panel (Active during "+ Add Junction Tool")
    if (m_isAddingJunction) {
        float jbW = 380.0f * m_uiScale;
        float itemH = 26.0f * m_uiScale;
        int listCount = std::max(1, (int)m_pendingJunctionConns.size());
        float jbH = 75.0f * m_uiScale + (float)listCount * itemH + 46.0f * m_uiScale;
        float jbX = ((float)screenW - jbW) * 0.5f;
        float jbY = 50.0f * m_uiScale;

        UI::panel("JUNCTION BUILDER", jbX, jbY, jbW, jbH);

        float jy = jbY + 34.0f * m_uiScale;
        std::string countTitle = "Selected Endpoints (" + std::to_string(m_pendingJunctionConns.size()) + "):";
        UI::label(countTitle, jbX + 14.0f * m_uiScale, jy, 0.95f * m_uiScale, glm::vec4(0.85f, 0.9f, 1.0f, 1.0f));
        jy += 22.0f * m_uiScale;

        if (m_pendingJunctionConns.empty()) {
            UI::label("(Click near road ends to select connection points)", jbX + 14.0f * m_uiScale, jy, 0.82f * m_uiScale, glm::vec4(0.65f, 0.65f, 0.7f, 1.0f));
            jy += itemH;
        } else {
            for (size_t i = 0; i < m_pendingJunctionConns.size(); ++i) {
                const auto& conn = m_pendingJunctionConns[i];
                std::string segName = (conn.segmentIdx >= 0 && conn.segmentIdx < (int)m_segments.size())
                                      ? m_segments[conn.segmentIdx].name : ("Seg #" + std::to_string(conn.segmentIdx));
                if (segName.length() > 22) segName = segName.substr(0, 20) + "..";
                std::string connDesc = "#" + std::to_string(i + 1) + " " + segName + (conn.isStart ? " [Start]" : " [End]");

                UI::label(connDesc, jbX + 18.0f * m_uiScale, jy + 3.0f * m_uiScale, 0.88f * m_uiScale, glm::vec4(0.9f, 0.95f, 0.8f, 1.0f));
                if (UI::button("X", jbX + jbW - 40.0f * m_uiScale, jy, 26.0f * m_uiScale, 20.0f * m_uiScale)) {
                    m_pendingJunctionConns.erase(m_pendingJunctionConns.begin() + i);
                    break;
                }
                jy += itemH;
            }
        }

        jy += 8.0f * m_uiScale;
        bool canCreate = (m_pendingJunctionConns.size() >= 2);
        float halfW = (jbW - 36.0f * m_uiScale) * 0.5f;
        std::string createText = canCreate ? ("Create Junction (" + std::to_string(m_pendingJunctionConns.size()) + "-way)") : "Select >= 2 Roads";
        if (UI::button(createText, jbX + 14.0f * m_uiScale, jy, halfW, 28.0f * m_uiScale, canCreate)) {
            createJunctionFromPending();
            return;
        }
        if (UI::button("Cancel [Esc]", jbX + 22.0f * m_uiScale + halfW, jy, halfW, 28.0f * m_uiScale)) {
            m_isAddingJunction = false;
            m_pendingJunctionConns.clear();
        }
    }

    // 6. Toast / Status Message at bottom
    if (m_statusTimer > 0.0f) {
        float toastW = (float)m_statusMessage.length() * 9.0f * m_uiScale + 28.0f * m_uiScale;
        float toastH = 28.0f * m_uiScale;
        float toastX = ((float)screenW - toastW) * 0.5f;
        float toastY = (float)screenH - toastH - 12.0f * m_uiScale;

        UI::drawRect(toastX, toastY, toastW, toastH, glm::vec4(0.12f, 0.14f, 0.18f, 0.95f));
        UI::drawRectOutline(toastX, toastY, toastW, toastH, 1.0f, glm::vec4(0.2f, 0.7f, 1.0f, 0.8f));
        UI::drawText(m_statusMessage, toastX + 14.0f * m_uiScale, toastY + 9.0f * m_uiScale, 1.0f * m_uiScale, glm::vec4(0.95f, 0.95f, 0.98f, 1.0f));
    }
}

void RoadEditor::setViewMode(EditorViewMode mode) {
    if (mode == EditorViewMode::DriveTest) {
        enterDriveMode(m_selectedSegmentIdx);
    } else {
        if (m_viewMode == EditorViewMode::DriveTest) {
            exitDriveMode();
        }
        m_viewMode = mode;
    }
}

void RoadEditor::setAddingJunction(bool adding) {
    m_isAddingJunction = adding;
    if (!adding) {
        m_pendingJunctionConns.clear();
    }
}

void RoadEditor::cancelAddingJunction() {
    m_isAddingJunction = false;
    m_pendingJunctionConns.clear();
}

void RoadEditor::selectSegment(int idx) {
    if (idx >= 0 && idx < (int)m_segments.size()) {
        m_selectedSegmentIdx = idx;
        m_selectedJunctionIdx = -1;
        m_selectedControlPointIdx = -1;
        m_selectedMesh = m_segments[idx].visualMesh;
    } else {
        m_selectedSegmentIdx = -1;
        m_selectedControlPointIdx = -1;
        m_selectedMesh = nullptr;
    }
}

void RoadEditor::selectJunction(int idx) {
    if (idx >= 0 && idx < (int)m_junctions.size()) {
        m_selectedJunctionIdx = idx;
        m_selectedSegmentIdx = -1;
        m_selectedControlPointIdx = -1;
        m_selectedMesh = m_junctions[idx].patchMesh;
    } else {
        m_selectedJunctionIdx = -1;
        m_selectedMesh = nullptr;
    }
}

const ProcessedRoadSegment* RoadEditor::getSelectedSegment() const {
    if (m_selectedSegmentIdx >= 0 && m_selectedSegmentIdx < (int)m_segments.size()) {
        return &m_segments[m_selectedSegmentIdx];
    }
    return nullptr;
}

ProcessedRoadSegment* RoadEditor::getSelectedSegment() {
    if (m_selectedSegmentIdx >= 0 && m_selectedSegmentIdx < (int)m_segments.size()) {
        return &m_segments[m_selectedSegmentIdx];
    }
    return nullptr;
}

const RoadJunction* RoadEditor::getSelectedJunction() const {
    if (m_selectedJunctionIdx >= 0 && m_selectedJunctionIdx < (int)m_junctions.size()) {
        return &m_junctions[m_selectedJunctionIdx];
    }
    return nullptr;
}

RoadJunction* RoadEditor::getSelectedJunction() {
    if (m_selectedJunctionIdx >= 0 && m_selectedJunctionIdx < (int)m_junctions.size()) {
        return &m_junctions[m_selectedJunctionIdx];
    }
    return nullptr;
}

void RoadEditor::insertControlPointBefore(int segIdx, int cpIdx) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    auto& seg = m_segments[segIdx];
    int idx = (cpIdx >= 0) ? cpIdx : 0;
    if (idx <= 0 && !seg.controlPoints.empty()) {
        glm::vec3 dir(0.0f, 0.0f, -1.0f);
        if (seg.controlPoints.size() >= 2) {
            dir = glm::normalize(seg.controlPoints[0] - seg.controlPoints[1]);
        }
        glm::vec3 newPt = seg.controlPoints.front() + dir * 15.0f;
        seg.controlPoints.insert(seg.controlPoints.begin(), newPt);
        seg.startJunctionId = -1;
        m_selectedControlPointIdx = 0;
    } else if (idx > 0 && idx < (int)seg.controlPoints.size()) {
        glm::vec3 mid = (seg.controlPoints[idx - 1] + seg.controlPoints[idx]) * 0.5f;
        seg.controlPoints.insert(seg.controlPoints.begin() + idx, mid);
        m_selectedControlPointIdx = idx;
    }
    rebuildSegmentGeometry(segIdx);
    if (m_autoSolveElevation) solveRoadElevations();
    m_statusMessage = "Inserted control point before #" + std::to_string(idx);
    m_statusTimer = 2.5f;
}

void RoadEditor::insertControlPointAfter(int segIdx, int cpIdx) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    auto& seg = m_segments[segIdx];
    int n = (int)seg.controlPoints.size();
    int idx = (cpIdx >= 0) ? cpIdx : (n - 1);
    if (idx >= n - 1 && n > 0) {
        glm::vec3 dir(0.0f, 0.0f, 1.0f);
        if (n >= 2) {
            dir = glm::normalize(seg.controlPoints[n - 1] - seg.controlPoints[n - 2]);
        }
        glm::vec3 newPt = seg.controlPoints.back() + dir * 15.0f;
        seg.controlPoints.push_back(newPt);
        seg.endJunctionId = -1;
        m_selectedControlPointIdx = (int)seg.controlPoints.size() - 1;
    } else if (idx >= 0 && idx < n - 1) {
        glm::vec3 mid = (seg.controlPoints[idx] + seg.controlPoints[idx + 1]) * 0.5f;
        seg.controlPoints.insert(seg.controlPoints.begin() + idx + 1, mid);
        m_selectedControlPointIdx = idx + 1;
    }
    rebuildSegmentGeometry(segIdx);
    if (m_autoSolveElevation) solveRoadElevations();
    m_statusMessage = "Inserted control point after #" + std::to_string(idx);
    m_statusTimer = 2.5f;
}

void RoadEditor::deleteControlPoint(int segIdx, int cpIdx) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    auto& seg = m_segments[segIdx];
    if (cpIdx >= 0 && cpIdx < (int)seg.controlPoints.size() && seg.controlPoints.size() > 2) {
        seg.controlPoints.erase(seg.controlPoints.begin() + cpIdx);
        m_selectedControlPointIdx = -1;
        rebuildSegmentGeometry(segIdx);
        if (m_autoSolveElevation) solveRoadElevations();
        m_statusMessage = "Deleted control point.";
        m_statusTimer = 2.5f;
    }
}

void RoadEditor::setControlPointElevation(int segIdx, int cpIdx, float elevation) {
    if (segIdx < 0 || segIdx >= (int)m_segments.size()) return;
    auto& seg = m_segments[segIdx];
    if (cpIdx >= 0 && cpIdx < (int)seg.controlPoints.size()) {
        if (m_isQuadsphereChunk) {
            glm::vec3 enu = glm::vec3(m_R_ecef_to_enu * glm::vec4(seg.controlPoints[cpIdx], 1.0f));
            enu.y = elevation;
            seg.controlPoints[cpIdx] = glm::vec3(m_R_enu_to_ecef * glm::vec4(enu, 1.0f));
        } else {
            seg.controlPoints[cpIdx].y = elevation;
        }
        rebuildSegmentGeometry(segIdx);
        if (m_autoSolveElevation) solveRoadElevations();
    }
}

void RoadEditor::applyAashtoStandards() {
    rebuildRoadGeometry();
    m_statusMessage = "Applied AASHTO standards to all road segments.";
    m_statusTimer = 3.0f;
}

int RoadEditor::getMeshVertexCount() const {
    return m_mergedMesh ? m_mergedMesh->vertexCount : 0;
}

int RoadEditor::getMeshTriangleCount() const {
    return m_mergedMesh ? (m_mergedMesh->indexCount / 3) : 0;
}

void RoadEditor::resetCameraFocus() {
    if (m_selectedSegmentIdx >= 0 && m_selectedSegmentIdx < (int)m_segments.size()) {
        const auto& seg = m_segments[m_selectedSegmentIdx];
        if (!seg.controlPoints.empty()) {
            glm::vec3 center(0.0f);
            for (const auto& pt : seg.controlPoints) center += pt;
            center /= (float)seg.controlPoints.size();
            m_cameraFocus = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(center, 1.0f)) : center;
        }
    } else if (m_selectedJunctionIdx >= 0 && m_selectedJunctionIdx < (int)m_junctions.size()) {
        glm::vec3 pt = m_junctions[m_selectedJunctionIdx].position;
        m_cameraFocus = m_isQuadsphereChunk ? glm::vec3(m_R_ecef_to_enu * glm::vec4(pt, 1.0f)) : pt;
    } else {
        m_cameraFocus = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    m_orbitDistance = 220.0f;
    m_orbitPitch = 45.0f;
}

void RoadEditor::reloadActiveChunk() {
    if (m_isQuadsphereChunk) {
        loadChunk(m_activeFace, m_activeChunkX, m_activeChunkZ);
    } else {
        loadChunk(m_activeChunkX, m_activeChunkZ);
    }
}

void RoadEditor::createNewSegment() {
    ProcessedRoadSegment seg;
    seg.id = static_cast<uint32_t>(m_segments.size() + 1);
    seg.name = "New Road";
    seg.highway = "residential";
    seg.lanes = 2;
    seg.laneWidth = 3.65f;
    seg.shoulderWidth = 1.50f;
    seg.sampleStep = 2.50f;
    seg.layer = 0;
    seg.face = m_activeFace;
    seg.chunkX = m_activeChunkX;
    seg.chunkZ = m_activeChunkZ;
    seg.chunkCenterECEF = m_chunkCenterECEF;
    seg.in500mChunk = true;

    // Create 3 initial control points centered around camera focus
    glm::vec3 center = m_cameraFocus;
    glm::vec3 p0 = center - glm::vec3(25.0f, 0.0f, 0.0f);
    glm::vec3 p1 = center;
    glm::vec3 p2 = center + glm::vec3(25.0f, 0.0f, 0.0f);

    if (m_isQuadsphereChunk) {
        seg.controlPoints.push_back(glm::vec3(m_R_enu_to_ecef * glm::vec4(p0, 1.0f)));
        seg.controlPoints.push_back(glm::vec3(m_R_enu_to_ecef * glm::vec4(p1, 1.0f)));
        seg.controlPoints.push_back(glm::vec3(m_R_enu_to_ecef * glm::vec4(p2, 1.0f)));
    } else {
        seg.controlPoints.push_back(p0);
        seg.controlPoints.push_back(p1);
        seg.controlPoints.push_back(p2);
    }

    m_segments.push_back(seg);
    int newIdx = static_cast<int>(m_segments.size() - 1);
    rebuildSegmentGeometry(newIdx);
    selectSegment(newIdx);
    m_statusMessage = "Created new road segment.";
    m_statusTimer = 3.0f;
}

void RoadEditor::testDriveSelectedRoad() {
    enterDriveMode(m_selectedSegmentIdx);
}
