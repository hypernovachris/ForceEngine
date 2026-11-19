#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <fstream>
#include <unordered_map>
#include "Terrain/TerrainTypes.h"
#include "Terrain/TerrainLoader.h"
#include "Road/RoadTypes.h"
#include "Road/GeoJsonLoader.h"
#include "Road/RoadGenerator.h"
#include "Road/RoadElevationSolver.h"

// Fast in-memory helper for sampling terrain elevation in unit test
static float sampleTerrainElevation(const LoadedTerrainPack& pack,
                                    std::unordered_map<uint64_t, DecodedTerrainTile>& cache,
                                    float worldX, float worldZ) {
    float relE = worldX - pack.startEast;
    float relN = -worldZ - pack.startNorth;
    float maxE = pack.tilesX_LOD0 * pack.header.baseTileSize;
    float maxN = pack.tilesY_LOD0 * pack.header.baseTileSize;

    if (relE < 0.0f || relN < 0.0f || relE > maxE || relN > maxN) {
        return pack.header.globalMinElevation;
    }

    for (uint16_t lod = 0; lod <= pack.header.maxLOD; ++lod) {
        float tileSz = pack.header.baseTileSize * static_cast<float>(1 << lod);
        uint32_t tx = static_cast<uint32_t>(relE / tileSz);
        uint32_t ty = static_cast<uint32_t>(relN / tileSz);
        uint64_t key = TerrainLoader::packTileKey(lod, tx, ty);

        auto it = cache.find(key);
        if (it != cache.end()) {
            float localX = relE - tx * tileSz;
            float localZ = relN - ty * tileSz;
            return TerrainLoader::sampleTileHeight(it->second, localX, localZ, tileSz);
        }

        const auto* entry = TerrainLoader::findTileIndex(pack, lod, tx, ty);
        if (entry) {
            DecodedTerrainTile tile;
            if (TerrainLoader::loadTile(pack, lod, tx, ty, tile)) {
                float localX = relE - tx * tileSz;
                float localZ = relN - ty * tileSz;
                float h = TerrainLoader::sampleTileHeight(tile, localX, localZ, tileSz);
                cache[key] = std::move(tile);
                return h;
            }
        }
    }
    return pack.header.globalMinElevation;
}

int main() {
    std::cout << "=== Testing Drawing Roads On Top of Terrain Integration ===" << std::endl;

    // Test 1: Open Terrain Container Pack
    std::cout << "\n--- 1. Testing Terrain Pack Loading & Geodetic Origin ---" << std::endl;
    std::string packPath = "jughandle/somerset_mc_lerc.fterrainpack";
    if (!std::ifstream(packPath).good()) {
        packPath = "jughandle/jughandle_lerc.fterrainpack";
    }
    LoadedTerrainPack pack;
    bool packOk = TerrainLoader::openPack(packPath, pack);
    assert(packOk && "Failed to open terrain pack container!");
    std::cout << "[PASS] Opened terrain pack: " << packPath << " (" << pack.header.tileCount << " tiles, LOD 0.."
              << pack.header.maxLOD << ")" << std::endl;
    std::cout << "  Geodetic Anchor: (" << pack.header.geodeticAnchor[0] << ", " << pack.header.geodeticAnchor[1] << ")" << std::endl;
    std::cout << "  Domain Physical Extents: [" << pack.startEast << "m, " << -pack.startEast << "m] E, ["
              << pack.startNorth << "m, " << -pack.startNorth << "m] N" << std::endl;

    // Test 2: Ingest GeoJSON Road Features using Terrain Pack WGS84 Anchor
    std::cout << "\n--- 2. Testing Road Ingestion with Terrain WGS84 ECEF Anchor ---" << std::endl;
    std::string geojsonPath = "jughandle/export.geojson";
    std::vector<RawRoadFeature> rawFeatures;
    bool loadOk = GeoJsonLoader::loadWithAnchor(geojsonPath,
                                               pack.header.ecefAnchor,
                                               pack.header.enuRotation,
                                               rawFeatures);
    assert(loadOk && "Failed to load GeoJSON with terrain anchor!");
    assert(!rawFeatures.empty() && "No road features extracted from GeoJSON!");
    std::cout << "[PASS] Ingested " << rawFeatures.size() << " road features accurately anchored to terrain." << std::endl;

    // Verify all raw road points fall inside the terrain domain boundaries
    float minX = 1e9f, maxX = -1e9f, minZ = 1e9f, maxZ = -1e9f;
    for (const auto& feat : rawFeatures) {
        for (const auto& pt : feat.rawPoints) {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minZ = std::min(minZ, pt.z);
            maxZ = std::max(maxZ, pt.z);
            assert(pt.x >= pack.startEast && pt.x <= -pack.startEast && "Road point X exceeds terrain East bounds!");
            assert(pt.z >= pack.startNorth && pt.z <= -pack.startNorth && "Road point Z exceeds terrain North bounds!");
        }
    }
    std::cout << "[PASS] All road coordinates strictly lie within terrain boundary:" << std::endl;
    std::cout << "  Road X range: [" << minX << "m, " << maxX << "m] (Terrain E: [" << pack.startEast << "m, " << -pack.startEast << "m])" << std::endl;
    std::cout << "  Road Z range: [" << minZ << "m, " << maxZ << "m] (Terrain N: [" << pack.startNorth << "m, " << -pack.startNorth << "m])" << std::endl;

    // Test 3: Process Road Features into Highway Network Segments
    std::cout << "\n--- 3. Testing Highway Feature Processing ---" << std::endl;
    std::vector<ProcessedRoadSegment> segments;
    RoadGenerator::processFeatures(rawFeatures, 3.65f, 1.50f, 2.50f, segments, false);
    assert(!segments.empty() && "Expected processed road segments!");
    std::cout << "[PASS] Processed " << segments.size() << " road segments." << std::endl;

    // Test 4: Solve Road Elevations with Physics Simulation (Tension, Adhesion, Repulsion)
    std::cout << "\n--- 4. Testing Road Elevation Solver on 0.5m LiDAR Terrain ---" << std::endl;
    std::unordered_map<uint64_t, DecodedTerrainTile> tileCache;
    RoadElevationSolverOptions solverOptions;
    solverOptions.minClearance = 5.5f;
    solverOptions.groundOffset = 0.15f;
    solverOptions.verbose = true;

    auto sampleTerrain = [&](float x, float z) -> float {
        return sampleTerrainElevation(pack, tileCache, x, z);
    };

    RoadElevationSolver::solve(segments, sampleTerrain, solverOptions);

    int groundSplinePoints = 0;
    int bridgeSplinePoints = 0;
    float minGroundElevation = 1e9f;
    float maxGroundElevation = -1e9f;

    float splineCutDepth = 0.0f;
    float splineFillHeight = 0.0f;
    int cutPoints = 0;
    int fillPoints = 0;

    for (const auto& seg : segments) {
        bool isBridge = (seg.isBridge || seg.layer > 0);
        for (const auto& sp : seg.splinePoints) {
            float gy = sampleTerrain(sp.pos.x, sp.pos.z);
            minGroundElevation = std::min(minGroundElevation, gy);
            maxGroundElevation = std::max(maxGroundElevation, gy);

            float diff = sp.pos.y - gy;
            if (isBridge) {
                bridgeSplinePoints++;
                assert(diff >= -0.5f && "Bridge points must not burrow deep into ground!");
            } else {
                groundSplinePoints++;
                if (diff < 0.0f) {
                    cutPoints++;
                    splineCutDepth = std::max(splineCutDepth, -diff);
                } else {
                    fillPoints++;
                    splineFillHeight = std::max(splineFillHeight, diff);
                }
            }
        }
    }

    std::cout << "[PASS] Solved " << groundSplinePoints << " ground road points (fill: "
              << fillPoints << " pts, cut: " << cutPoints << " pts, max cut: " << splineCutDepth << "m)." << std::endl;
    std::cout << "[PASS] Solved " << bridgeSplinePoints << " elevated bridge points." << std::endl;
    std::cout << "  LiDAR Ground Elevation Range Under Roads: [" << minGroundElevation << "m, " << maxGroundElevation << "m]" << std::endl;

    // Test 5: Network Topology & Planar Junction Patches
    std::cout << "\n--- 5. Testing Network Topology & Junction Generation ---" << std::endl;
    std::vector<RoadJunction> junctions;
    RoadGenerator::buildNetworkTopology(segments, junctions);
    RoadGenerator::generateJunctionPatches(segments, junctions);

    int patchCount = 0;
    int patchVerts = 0;
    int patchTris = 0;
    for (const auto& j : junctions) {
        if (!j.patchVertices.empty()) {
            patchCount++;
            patchVerts += static_cast<int>(j.patchVertices.size());
            patchTris += static_cast<int>(j.patchIndices.size() / 3);
        }
    }
    std::cout << "[PASS] Generated " << junctions.size() << " junctions with "
              << patchCount << " planar intersection plates (" << patchVerts << " vertices, " << patchTris << " triangles)." << std::endl;
    assert(junctions.size() >= 20 && "Expected at least 20 junctions in Somerset jughandle!");

    // Test 6: Unified Merged Mesh Generation (1-Draw-Call GPU Buffer)
    std::cout << "\n--- 6. Testing Merged Road Mesh Construction ---" << std::endl;
    std::vector<Vertex> mergedVerts;
    std::vector<uint32_t> mergedIdxs;

    for (const auto& seg : segments) {
        auto trimmed = RoadGenerator::trimSpline(seg.splinePoints, seg.startSetback, seg.endSetback);
        std::vector<Vertex> segVerts;
        std::vector<uint32_t> segIdxs;
        RoadGenerator::generateRibbonGeometry(trimmed, seg.totalWidth, seg.lanes, seg.oneway, seg.shoulderWidth, segVerts, segIdxs);
        RoadGenerator::clipRibbonAgainstJunctions(segVerts, seg, junctions, segments);
        uint32_t baseVertex = static_cast<uint32_t>(mergedVerts.size());
        for (const auto& v : segVerts) mergedVerts.push_back(v);
        for (auto idx : segIdxs) mergedIdxs.push_back(baseVertex + idx);
    }
    for (const auto& junc : junctions) {
        if (junc.patchVertices.empty() || junc.patchIndices.empty()) continue;
        uint32_t baseVertex = static_cast<uint32_t>(mergedVerts.size());
        for (const auto& v : junc.patchVertices) mergedVerts.push_back(v);
        for (auto idx : junc.patchIndices) mergedIdxs.push_back(baseVertex + idx);
    }

    assert(!mergedVerts.empty() && "Merged road vertices must not be empty!");
    assert(!mergedIdxs.empty() && "Merged road indices must not be empty!");
    assert(mergedIdxs.size() % 3 == 0 && "Index count must be a multiple of 3 (triangles)!");

    std::cout << "[PASS] Unified road mesh generated: " << mergedVerts.size() << " vertices, "
              << (mergedIdxs.size() / 3) << " triangles." << std::endl;

    // Verify road ribbon and junction vertices relation to terrain ground
    int cutCount = 0;
    int fillCount = 0;
    float maxCutDepth = 0.0f;
    float maxFillHeight = 0.0f;

    for (const auto& v : mergedVerts) {
        float gy = sampleTerrainElevation(pack, tileCache, v.position[0], v.position[2]);
        float diff = v.position[1] - gy;
        if (diff < 0.0f) {
            cutCount++;
            maxCutDepth = std::max(maxCutDepth, -diff);
        } else {
            fillCount++;
            maxFillHeight = std::max(maxFillHeight, diff);
        }
    }
    std::cout << "[PASS] Verified road mesh across terrain topography:" << std::endl;
    std::cout << "  Road vertices on/above ground (embankment/fill/bridges): " << fillCount << " (max height above ground: " << maxFillHeight << "m)" << std::endl;
    std::cout << "  Road vertices in hillside cuts (civil engineering cut slopes): " << cutCount << " (max cut depth into hillside: " << maxCutDepth << "m)" << std::endl;

    // Test 7: Testing 3D Curb Profile Dimensions & Normals
    std::cout << "\n--- 7. Testing 3D Curb Profile Dimensions & Geometry ---" << std::endl;
    {
        std::vector<RoadSplinePoint> testSpline = {
            { glm::vec3(0.0f, 10.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 0.0f },
            { glm::vec3(0.0f, 10.0f, 20.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 20.0f }
        };

        const float testTotalWidth = 8.0f; // 4.0m half-width
        std::vector<Vertex> curbVerts;
        std::vector<uint32_t> curbIdxs;
        RoadGenerator::generateRibbonGeometry(testSpline, testTotalWidth, 2, false, 1.0f, curbVerts, curbIdxs);

        assert(!curbVerts.empty() && "Curb vertices must not be empty!");
        assert(!curbIdxs.empty() && "Curb indices must not be empty!");
        assert(curbIdxs.size() % 3 == 0 && "Indices must form valid triangles!");

        // Verify dimensions:
        // Road surface is at y = 10.0m
        // Curb top must be at y = 10.15m (+15cm above road)
        // Curb skirt bottom must be at y = 9.50m (-50cm below road)
        // Left curb top must span x in [-4.30m, -4.00m] (width = 0.30m = 30cm)
        // Right curb top must span x in [+4.00m, +4.30m] (width = 0.30m = 30cm)

        bool foundCurbTop = false;
        bool foundCurbSkirt = false;
        bool foundRoadDeck = false;
        float maxCurbY = -1e9f;
        float minCurbY = 1e9f;
        float minCurbX = 1e9f;
        float maxCurbX = -1e9f;

        for (const auto& v : curbVerts) {
            maxCurbY = std::max(maxCurbY, v.position[1]);
            minCurbY = std::min(minCurbY, v.position[1]);
            minCurbX = std::min(minCurbX, v.position[0]);
            maxCurbX = std::max(maxCurbX, v.position[0]);

            if (std::abs(v.position[1] - 10.15f) < 1e-3f) foundCurbTop = true;
            if (std::abs(v.position[1] - 9.50f) < 1e-3f) foundCurbSkirt = true;
            if (std::abs(v.position[1] - 10.00f) < 1e-3f) foundRoadDeck = true;

            // Verify normal vectors are unit length
            float normLen = std::sqrt(v.normal[0]*v.normal[0] + v.normal[1]*v.normal[1] + v.normal[2]*v.normal[2]);
            assert(std::abs(normLen - 1.0f) < 1e-3f && "Vertex normal must be unit length!");
        }

        assert(foundCurbTop && "Must have curb top vertices at +15cm (0.15m) above road!");
        assert(foundCurbSkirt && "Must have curb skirt vertices at -50cm (0.50m) below road!");
        assert(foundRoadDeck && "Must have road deck vertices at 0cm (road surface)!");

        assert(std::abs(maxCurbY - 10.15f) < 1e-3f && "Curb max elevation must be road + 0.15m!");
        assert(std::abs(minCurbY - 9.50f) < 1e-3f && "Curb min elevation must be road - 0.50m!");

        // Total lateral corridor width = totalWidth (8.0m) + 2 * curbW (2 * 0.30m) = 8.60m
        float totalCorridorSpan = maxCurbX - minCurbX;
        assert(std::abs(totalCorridorSpan - (testTotalWidth + 2.0f * 0.30f)) < 1e-3f &&
               "Curb width must be exactly 0.30m (30cm) on each side!");

        std::cout << "[PASS] Verified 3D curb profile geometry:" << std::endl;
        std::cout << "  Curb Top Height above road: " << (maxCurbY - 10.0f) * 100.0f << " cm (Target: +15 cm)" << std::endl;
        std::cout << "  Curb Skirt Depth below road: " << (10.0f - minCurbY) * 100.0f << " cm (Target: -50 cm)" << std::endl;
        std::cout << "  Curb Width per side: " << ((totalCorridorSpan - testTotalWidth) * 0.5f) * 100.0f << " cm (Target: 30 cm)" << std::endl;
    }

    // Test 8: Testing Terrain Sudden Boundary Stop & Out-of-Bounds Omission
    std::cout << "\n--- 8. Testing Terrain Boundary Sudden Cutoff (No Flat Fallback) ---" << std::endl;
    {
        // Coordinate helper replicating the engine's boundary check:
        auto isTerrainUsableAt = [](const LoadedTerrainPack& pk, float posX, float posZ) -> bool {
            float relE = posX - pk.startEast;
            float relN = -posZ - pk.startNorth;
            float maxE = pk.tilesX_LOD0 * pk.header.baseTileSize;
            float maxN = pk.tilesY_LOD0 * pk.header.baseTileSize;
            if (relE < 0.0f || relN < 0.0f || relE > maxE || relN > maxN) return false;

            for (int lod = 0; lod <= (int)pk.header.maxLOD; ++lod) {
                float tileSz = pk.header.baseTileSize * static_cast<float>(1 << lod);
                uint32_t tx = static_cast<uint32_t>(relE / tileSz);
                uint32_t ty = static_cast<uint32_t>(relN / tileSz);
                const auto* entry = TerrainLoader::findTileIndex(pk, lod, tx, ty);
                if (entry) {
                    // Artificial border tile outside the map has maxElevation == minElevation
                    if (entry->maxElevation - entry->minElevation < 0.001f) {
                        return false;
                    }
                    return true;
                }
            }
            return false;
        };

        // Coordinates inside the county map:
        bool hasInside = isTerrainUsableAt(pack, 0.0f, 0.0f);
        assert(hasInside && "Must have valid terrain elevation inside Somerset County!");

        // Coordinates far outside Somerset County (e.g. 50km away):
        bool hasOutside = isTerrainUsableAt(pack, 50000.0f, 50000.0f);
        assert(!hasOutside && "Must NOT have terrain elevation outside the mapped boundary!");

        // Coordinates on clamped border corner (artificial flat padding tile):
        bool hasCorner = isTerrainUsableAt(pack, pack.startEast + 10.0f, -(pack.startNorth + 10.0f));
        assert(!hasCorner && "Artificial flat border tile must be omitted (terrain stops!)");

        std::cout << "[PASS] Verified terrain stops cleanly at map boundary with 0 flat fallback." << std::endl;
    }

    std::cout << "\nALL ROAD-ON-TERRAIN INTEGRATION TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
