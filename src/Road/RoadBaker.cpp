#include "Road/RoadBaker.h"
#include "Road/GeoJsonLoader.h"
#include "Road/RoadGenerator.h"
#include "Road/RoadElevationSolver.h"
#include "Road/RoadBinary.h"
#include "Terrain/TerrainLoader.h"
#include "Terrain/QuadsphereLoader.h"
#include "Terrain/QuadsphereMath.h"
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <filesystem>

static float sampleQuadsphereElevation(const LoadedQuadspherePack& pack,
                                       std::unordered_map<uint64_t, DecodedQuadsphereTile>& cache,
                                       double latDeg, double lonDeg) {
    if (!pack.isValid() || pack.indexTable.empty()) return 0.0f;

    uint8_t face; double u, v;
    Quadsphere::geodeticToQuadsphere(latDeg, lonDeg, face, u, v);

    for (int lod = pack.header.maxLOD; lod >= pack.header.minLOD; --lod) {
        double tilesPerSide = static_cast<double>(1ULL << lod);
        double span = 2.0 / tilesPerSide;

        int tx = static_cast<int>(std::floor((u + 1.0) / span));
        int ty = static_cast<int>(std::floor((v + 1.0) / span));
        if (tx < 0 || ty < 0 || tx >= (int)tilesPerSide || ty >= (int)tilesPerSide) continue;

        const auto* entry = QuadsphereLoader::findTileIndex(pack, face, lod, tx, ty);
        if (!entry) continue;

        uint64_t key = entry->key;
        auto it = cache.find(key);
        if (it == cache.end()) {
            DecodedQuadsphereTile tile;
            if (QuadsphereLoader::loadTile(pack, face, lod, tx, ty, tile)) {
                it = cache.emplace(key, std::move(tile)).first;
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
        return h0 * (1.0f - ry) + h1 * ry;
    }
    return 15.0f;
}

static float samplePackElevation(const LoadedTerrainPack& pack,
                                 std::unordered_map<uint64_t, DecodedTerrainTile>& cache,
                                 float posX, float posZ) {
    float relE = posX - pack.startEast;
    float relN = -posZ - pack.startNorth;

    for (int lod = 0; lod <= static_cast<int>(pack.header.maxLOD); ++lod) {
        float tileSz = pack.header.baseTileSize * static_cast<float>(1 << lod);
        if (relE < 0.0f || relN < 0.0f) continue;
        uint32_t tx = static_cast<uint32_t>(relE / tileSz);
        uint32_t ty = static_cast<uint32_t>(relN / tileSz);

        uint64_t key = TerrainLoader::packTileKey(lod, tx, ty);
        auto it = cache.find(key);
        if (it != cache.end()) {
            return TerrainLoader::sampleTileHeight(it->second, relE - tx * tileSz, relN - ty * tileSz, tileSz);
        }

        const auto* entry = TerrainLoader::findTileIndex(pack, lod, tx, ty);
        if (entry) {
            DecodedTerrainTile tile;
            if (TerrainLoader::loadTile(pack, lod, tx, ty, tile)) {
                float h = TerrainLoader::sampleTileHeight(tile, relE - tx * tileSz, relN - ty * tileSz, tileSz);
                cache[key] = std::move(tile);
                return h;
            }
        }
    }
    return pack.header.globalMinElevation;
}

RoadBakeResult RoadBaker::bake(const RoadBakeOptions& options) {
    RoadBakeResult result;

    if (options.verbose) {
        std::cout << "========================================\n"
                  << "        FORCEENGINE ROAD BAKER          \n"
                  << "========================================\n"
                  << "GeoJSON Input:    " << options.geoJsonPath << "\n"
                  << "Terrain Container: " << (options.terrainPackPath.empty() ? "[None - Flat Plane]" : options.terrainPackPath) << "\n"
                  << "Output Binary:    " << options.outputPath << "\n"
                  << "Filter 500m:      " << (options.filter500mOnly ? "YES" : "NO (All)") << "\n"
                  << "Ground Offset:    " << options.groundOffset << " m\n"
                  << "Bridge Clearance: " << options.bridgeClearance << " m\n"
                  << "----------------------------------------" << std::endl;
    }

    // 1. Verify GeoJSON file exists
    std::ifstream geoFile(options.geoJsonPath);
    if (!geoFile.good()) {
        result.errorMessage = "Failed to open GeoJSON input file: " + options.geoJsonPath;
        if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
        return result;
    }
    geoFile.close();

    // 2. Open optional terrain pack
    LoadedTerrainPack pack;
    LoadedQuadspherePack quadPack;
    bool hasTerrain = false;
    bool isQuadsphere = false;
    double centerLon = -74.515211, centerLat = 40.592676;

    if (!options.terrainPackPath.empty() && std::ifstream(options.terrainPackPath).good()) {
        if (options.terrainPackPath.rfind(".fquadspherepack") != std::string::npos) {
            isQuadsphere = QuadsphereLoader::openPack(options.terrainPackPath, quadPack);
            hasTerrain = isQuadsphere;
            if (isQuadsphere && options.verbose) {
                std::cout << "RoadBaker: Loaded Quadsphere pack (" << quadPack.header.tileCount << " tiles, LOD "
                          << quadPack.header.minLOD << ".." << quadPack.header.maxLOD << ")" << std::endl;
            }
        } else {
            hasTerrain = TerrainLoader::openPack(options.terrainPackPath, pack);
            if (hasTerrain && options.verbose) {
                std::cout << "RoadBaker: Loaded terrain pack (" << pack.header.tileCount << " tiles, LOD "
                          << pack.header.minLOD << ".." << pack.header.maxLOD << ")" << std::endl;
            }
        }
    }

    // 3. Ingest GeoJSON features
    std::vector<RawRoadFeature> rawFeatures;
    if (hasTerrain && !isQuadsphere) {
        GeoJsonLoader::loadWithAnchor(options.geoJsonPath,
                                      pack.header.ecefAnchor,
                                      pack.header.enuRotation,
                                      rawFeatures);
    } else {
        GeoJsonLoader::load(options.geoJsonPath, rawFeatures, centerLon, centerLat);
    }

    result.rawFeatureCount = static_cast<uint32_t>(rawFeatures.size());
    if (rawFeatures.empty()) {
        result.errorMessage = "No valid road features found in GeoJSON: " + options.geoJsonPath;
        if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
        return result;
    }

    if (options.verbose) {
        std::cout << "RoadBaker: Ingested " << rawFeatures.size() << " raw road features." << std::endl;
    }

    // 4. Generate spline centerlines and standard AASHTO cross-sections
    std::vector<ProcessedRoadSegment> segments;
    RoadGenerator::processFeatures(rawFeatures, 3.65f, 1.50f, 2.50f, segments, false);

    result.processedSegmentCount = static_cast<uint32_t>(segments.size());

    // 5. Solve road elevations using physics simulation (tension, adhesion, repulsion)
    std::unordered_map<uint64_t, DecodedTerrainTile> tileCache;
    std::unordered_map<uint64_t, DecodedQuadsphereTile> quadTileCache;
    if (hasTerrain && options.drapeOnTerrain) {
        RoadElevationSolverOptions solverOptions;
        solverOptions.minClearance = options.bridgeClearance;
        solverOptions.groundOffset = options.groundOffset;
        solverOptions.verbose = options.verbose;

        std::function<float(float, float)> sampleTerrain;
        if (isQuadsphere) {
            const double degToRad = 3.14159265358979323846 / 180.0;
            const double m_per_lat = 111139.0;
            const double m_per_lon = 111139.0 * std::cos(centerLat * degToRad);

            sampleTerrain = [&](float x, float z) -> float {
                double lat = centerLat - (z / m_per_lat);
                double lon = centerLon + (x / m_per_lon);
                return sampleQuadsphereElevation(quadPack, quadTileCache, lat, lon);
            };
        } else {
            sampleTerrain = [&](float x, float z) -> float {
                return samplePackElevation(pack, tileCache, x, z);
            };
        }

        RoadElevationSolver::solve(segments, sampleTerrain, solverOptions);
    }

    // 6. Build network topology and detect junctions
    RoadGenerator::collapseShortJunctionSegments(segments, 26.0f);
    std::vector<RoadJunction> junctions;
    RoadGenerator::buildNetworkTopology(segments, junctions);
    RoadGenerator::generateJunctionPatches(segments, junctions);

    result.junctionCount = static_cast<uint32_t>(junctions.size());

    // 7. Trim ribbons and compute geometry statistics
    for (auto& s : segments) {
        auto trimmed = RoadGenerator::trimSpline(s.splinePoints, s.startSetback, s.endSetback);
        std::vector<Vertex> verts;
        std::vector<uint32_t> idxs;
        RoadGenerator::generateRibbonGeometry(trimmed, s.totalWidth,
                                              s.lanes, s.oneway, s.shoulderWidth,
                                              verts, idxs);
        RoadGenerator::clipRibbonAgainstJunctions(verts, s, junctions, segments);

        result.totalVertices += static_cast<uint32_t>(verts.size());
        result.totalTriangles += static_cast<uint32_t>(idxs.size() / 3);
        result.totalLengthMeters += s.length;
    }

    // Add junction planar patch plates to geometry counts
    for (const auto& j : junctions) {
        result.totalVertices += static_cast<uint32_t>(j.patchVertices.size());
        result.totalTriangles += static_cast<uint32_t>(j.patchIndices.size() / 3);
    }

    // 8. Serialize to binary container (.froad)
    if (!RoadBinary::save500mChunk(options.outputPath, segments, junctions)) {
        result.errorMessage = "Failed to write compiled .froad binary to " + options.outputPath;
        if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
        return result;
    }

    std::ifstream outFile(options.outputPath, std::ios::binary | std::ios::ate);
    if (outFile.good()) {
        result.outputFileSize = static_cast<size_t>(outFile.tellg());
    }

    result.success = true;

    if (options.verbose) {
        std::cout << "----------------------------------------\n"
                  << "RoadBaker: Baking Completed Successfully!\n"
                  << "  Segments Baked:  " << result.processedSegmentCount << "\n"
                  << "  Junctions Built: " << result.junctionCount << "\n"
                  << "  Total Triangles: " << result.totalTriangles << "\n"
                  << "  Total Vertices:  " << result.totalVertices << "\n"
                  << "  Total Length:    " << (result.totalLengthMeters / 1000.0f) << " km\n"
                  << "  File Size:       " << (result.outputFileSize / 1024.0f) << " KB\n"
                  << "  Output File:     " << options.outputPath << "\n"
                  << "========================================" << std::endl;
    }

    return result;
}

MultiChunkBakeResult RoadBaker::bakeChunks(const MultiChunkBakeOptions& options) {
    MultiChunkBakeResult result;

    if (options.verbose) {
        std::cout << "========================================\n"
                  << "    FORCEENGINE MULTI-CHUNK ROAD BAKER   \n"
                  << "========================================\n"
                  << "GeoJSON Input:    " << options.geoJsonPath << "\n"
                  << "Terrain Container: " << (options.terrainPackPath.empty() ? "[None - Flat Plane]" : options.terrainPackPath) << "\n"
                  << "Output Directory: " << options.outputDir << "\n"
                  << "Chunk Size:       " << options.chunkSize << " m\n"
                  << "Halo Margin:      " << options.haloMargin << " m\n"
                  << "Ground Offset:    " << options.groundOffset << " m\n"
                  << "Bridge Clearance: " << options.bridgeClearance << " m\n"
                  << "----------------------------------------" << std::endl;
    }

    // 1. Verify GeoJSON exists
    std::ifstream geoFile(options.geoJsonPath);
    if (!geoFile.good()) {
        result.errorMessage = "Failed to open GeoJSON input file: " + options.geoJsonPath;
        if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
        return result;
    }
    geoFile.close();

    // 2. Ensure output directory exists
    std::error_code ec;
    std::filesystem::create_directories(options.outputDir, ec);
    if (ec) {
        result.errorMessage = "Failed to create output directory: " + options.outputDir + " (" + ec.message() + ")";
        if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
        return result;
    }

    // 3. Open terrain pack
    LoadedTerrainPack pack;
    LoadedQuadspherePack quadPack;
    bool hasTerrain = false;
    bool isQuadsphere = false;
    double centerLon = -74.515211, centerLat = 40.592676;

    if (!options.terrainPackPath.empty() && std::ifstream(options.terrainPackPath).good()) {
        if (options.terrainPackPath.rfind(".fquadspherepack") != std::string::npos) {
            isQuadsphere = QuadsphereLoader::openPack(options.terrainPackPath, quadPack);
            hasTerrain = isQuadsphere;
            if (isQuadsphere && options.verbose) {
                std::cout << "RoadBaker: Loaded Quadsphere pack (" << quadPack.header.tileCount << " tiles, LOD "
                          << quadPack.header.minLOD << ".." << quadPack.header.maxLOD << ")" << std::endl;
            }
        } else {
            hasTerrain = TerrainLoader::openPack(options.terrainPackPath, pack);
            if (hasTerrain && options.verbose) {
                std::cout << "RoadBaker: Loaded terrain pack (" << pack.header.tileCount << " tiles, LOD "
                          << pack.header.minLOD << ".." << pack.header.maxLOD << ")" << std::endl;
            }
        }
    }

    // 4. Ingest GeoJSON features
    std::vector<RawRoadFeature> rawFeatures;
    if (hasTerrain && !isQuadsphere) {
        GeoJsonLoader::loadWithAnchor(options.geoJsonPath,
                                      pack.header.ecefAnchor,
                                      pack.header.enuRotation,
                                      rawFeatures);
    } else {
        GeoJsonLoader::load(options.geoJsonPath, rawFeatures, centerLon, centerLat);
    }

    if (rawFeatures.empty()) {
        result.errorMessage = "No valid road features found in GeoJSON: " + options.geoJsonPath;
        if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
        return result;
    }

    if (options.verbose) {
        std::cout << "RoadBaker: Ingested " << rawFeatures.size() << " raw road features." << std::endl;
    }

    // 5. Generate continuous road corridors
    std::vector<ProcessedRoadSegment> fullSegments;
    RoadGenerator::processFeatures(rawFeatures, 3.65f, 1.50f, 2.50f, fullSegments, false, false);

    if (options.verbose) {
        std::cout << "RoadBaker: Processed " << fullSegments.size() << " road corridors." << std::endl;
    }

    if (isQuadsphere) {
        // Quadsphere Level 15 Domain Decomposition Pipeline (SPHERE.md Phase 4)
        std::map<std::tuple<uint8_t, uint32_t, uint32_t>, std::vector<ProcessedRoadSegment>> quadChunkSegments;
        RoadGenerator::splitSegmentsAcrossQuadsphere(fullSegments, 15, quadChunkSegments);

        if (quadChunkSegments.empty()) {
            result.errorMessage = "No road chunks produced after Quadsphere splitting.";
            if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
            return result;
        }

        std::unordered_map<uint64_t, DecodedQuadsphereTile> quadTileCache;
        auto sampleTerrainQuad = [&](double latDeg, double lonDeg) -> float {
            return sampleQuadsphereElevation(quadPack, quadTileCache, latDeg, lonDeg);
        };

        // Shared registry of solved seam boundary elevations (ECEF keyed)
        auto makeEcefSeamKey = [](const glm::dvec3& ecef) -> uint64_t {
            int64_t x = static_cast<int64_t>(std::round(ecef.x * 100.0));
            int64_t y = static_cast<int64_t>(std::round(ecef.y * 100.0));
            int64_t z = static_cast<int64_t>(std::round(ecef.z * 100.0));
            return (uint64_t)x * 73856093ULL ^ (uint64_t)y * 19349663ULL ^ (uint64_t)z * 83492791ULL;
        };
        std::unordered_map<uint64_t, float> seamSolvedElevations;

        for (auto& pair : quadChunkSegments) {
            uint8_t face = std::get<0>(pair.first);
            uint32_t tx = std::get<1>(pair.first);
            uint32_t ty = std::get<2>(pair.first);
            auto& curChunkSegs = pair.second;

            double minU, maxU, minV, maxV;
            Quadsphere::getTileUVBounds(15, tx, ty, minU, maxU, minV, maxV);
            double uCenter = (minU + maxU) * 0.5;
            double vCenter = (minV + maxV) * 0.5;
            glm::dvec3 chunkCenterECEF = Quadsphere::quadsphereToECEF(face, uCenter, vCenter, 0.0);

            // Dirichlet boundary condition: extract pinned nodes from already-solved neighbor seams
            RoadElevationSolverOptions solverOpts;
            solverOpts.minClearance = options.bridgeClearance;
            solverOpts.groundOffset = options.groundOffset;
            solverOpts.verbose = false;

            for (auto& s : curChunkSegs) {
                if (s.startNeighbor.hasNeighbor) {
                    auto key = makeEcefSeamKey(s.startNeighbor.boundaryPosECEF);
                    auto it = seamSolvedElevations.find(key);
                    if (it != seamSolvedElevations.end()) {
                        solverOpts.pinnedNodes.push_back({ glm::vec2(s.startNeighbor.boundaryPos.x, s.startNeighbor.boundaryPos.z), it->second });
                    }
                }
                if (s.endNeighbor.hasNeighbor) {
                    auto key = makeEcefSeamKey(s.endNeighbor.boundaryPosECEF);
                    auto it = seamSolvedElevations.find(key);
                    if (it != seamSolvedElevations.end()) {
                        solverOpts.pinnedNodes.push_back({ glm::vec2(s.endNeighbor.boundaryPos.x, s.endNeighbor.boundaryPos.z), it->second });
                    }
                }
            }

            // Solve elevations using Quadsphere Gauss-Seidel solver
            RoadElevationSolver::solveQuadsphere(curChunkSegs, sampleTerrainQuad, solverOpts);

            // Register solved boundary elevations
            for (auto& s : curChunkSegs) {
                if (s.startNeighbor.hasNeighbor) {
                    glm::dvec3 pECEF = s.chunkCenterECEF + glm::dvec3(s.controlPoints.front());
                    float h = static_cast<float>(glm::length(pECEF) - Quadsphere::EARTH_RADIUS);
                    seamSolvedElevations[makeEcefSeamKey(s.startNeighbor.boundaryPosECEF)] = h;
                }
                if (s.endNeighbor.hasNeighbor) {
                    glm::dvec3 pECEF = s.chunkCenterECEF + glm::dvec3(s.controlPoints.back());
                    float h = static_cast<float>(glm::length(pECEF) - Quadsphere::EARTH_RADIUS);
                    seamSolvedElevations[makeEcefSeamKey(s.endNeighbor.boundaryPosECEF)] = h;
                }
            }

            // Build network topology & junctions for this chunk
            RoadGenerator::collapseShortJunctionSegments(curChunkSegs, 26.0f);
            std::vector<RoadJunction> junctions;
            RoadGenerator::buildNetworkTopology(curChunkSegs, junctions);
            RoadGenerator::generateJunctionPatches(curChunkSegs, junctions, &chunkCenterECEF);

            // Save Quadsphere chunk binary (Version 3)
            std::string chunkFilename = options.outputDir + "/chunk_" + std::to_string(face) + "_" +
                                        std::to_string(tx) + "_" + std::to_string(ty) + ".froad";
            if (!RoadBinary::saveQuadsphereChunk(chunkFilename, face, 15, tx, ty, chunkCenterECEF,
                                                static_cast<float>(minU), static_cast<float>(maxU),
                                                static_cast<float>(minV), static_cast<float>(maxV),
                                                curChunkSegs, junctions)) {
                result.errorMessage = "Failed to write Quadsphere chunk binary: " + chunkFilename;
                if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
                return result;
            }

            result.totalChunks++;
            result.totalSegments += static_cast<uint32_t>(curChunkSegs.size());
            for (const auto& s : curChunkSegs) {
                result.totalLengthMeters += s.length;
                if (s.startNeighbor.hasNeighbor) result.totalCrossChunkConnections++;
                if (s.endNeighbor.hasNeighbor) result.totalCrossChunkConnections++;
            }

            if (options.verbose && (result.totalChunks % 100 == 0 || result.totalChunks == quadChunkSegments.size())) {
                std::cout << "RoadBaker: Baked Quadsphere [" << result.totalChunks << "/" << quadChunkSegments.size()
                          << "] chunks (" << result.totalSegments << " segments, "
                          << (result.totalLengthMeters / 1000.0f) << " km)..." << std::endl;
            }
        }

        result.success = true;
        if (options.verbose) {
            std::cout << "========================================\n"
                      << "RoadBaker: Quadsphere Multi-Chunk Bake Completed!\n"
                      << "  Total Chunks:           " << result.totalChunks << "\n"
                      << "  Total Segments:         " << result.totalSegments << "\n"
                      << "  Cross-Chunk Connections: " << result.totalCrossChunkConnections << "\n"
                      << "  Total Road Length:      " << (result.totalLengthMeters / 1000.0f) << " km\n"
                      << "  Output Directory:       " << options.outputDir << "\n"
                      << "========================================" << std::endl;
        }
        return result;
    }

    // 6. Split segments across the 512x512m chunk grid
    std::map<std::pair<int, int>, std::vector<ProcessedRoadSegment>> chunkSegments;
    RoadGenerator::splitSegmentsAcrossGrid(fullSegments, options.chunkSize, chunkSegments);

    if (chunkSegments.empty()) {
        result.errorMessage = "No road chunks produced after grid splitting.";
        if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
        return result;
    }

    std::unordered_map<uint64_t, DecodedTerrainTile> tileCache;
    std::unordered_map<uint64_t, DecodedQuadsphereTile> quadTileCache;

    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(centerLat * degToRad);

    auto sampleTerrain = [&](float x, float z) -> float {
        if (!hasTerrain) return 0.0f;
        if (isQuadsphere) {
            double lat = centerLat - (z / m_per_lat);
            double lon = centerLon + (x / m_per_lon);
            return sampleQuadsphereElevation(quadPack, quadTileCache, lat, lon);
        }
        return samplePackElevation(pack, tileCache, x, z);
    };

    // Shared registry of solved seam boundary elevations for Dirichlet boundary conditions
    std::map<std::pair<int, int>, float> seamSolvedElevations;
    auto makeSeamKey = [](float x, float z) -> std::pair<int, int> {
        return { static_cast<int>(std::round(x * 100.0f)), static_cast<int>(std::round(z * 100.0f)) };
    };

    float halfS = options.chunkSize * 0.5f;

    // 7. Solve and bake each chunk
    for (auto& pair : chunkSegments) {
        int cx = pair.first.first;
        int cz = pair.first.second;
        auto& curChunkSegs = pair.second;

        // Dirichlet boundary condition: extract pinned nodes from already-solved neighbor seams
        RoadElevationSolverOptions solverOpts;
        solverOpts.minClearance = options.bridgeClearance;
        solverOpts.groundOffset = options.groundOffset;
        solverOpts.verbose = false;

        for (auto& s : curChunkSegs) {
            if (s.startNeighbor.hasNeighbor) {
                auto key = makeSeamKey(s.startNeighbor.boundaryPos.x, s.startNeighbor.boundaryPos.z);
                auto it = seamSolvedElevations.find(key);
                if (it != seamSolvedElevations.end()) {
                    solverOpts.pinnedNodes.push_back({ glm::vec2(s.startNeighbor.boundaryPos.x, s.startNeighbor.boundaryPos.z), it->second });
                    s.startNeighbor.boundaryPos.y = it->second;
                    if (!s.splinePoints.empty()) s.splinePoints.front().pos.y = it->second;
                    if (!s.controlPoints.empty()) s.controlPoints.front().y = it->second;
                }
            }
            if (s.endNeighbor.hasNeighbor) {
                auto key = makeSeamKey(s.endNeighbor.boundaryPos.x, s.endNeighbor.boundaryPos.z);
                auto it = seamSolvedElevations.find(key);
                if (it != seamSolvedElevations.end()) {
                    solverOpts.pinnedNodes.push_back({ glm::vec2(s.endNeighbor.boundaryPos.x, s.endNeighbor.boundaryPos.z), it->second });
                    s.endNeighbor.boundaryPos.y = it->second;
                    if (!s.splinePoints.empty()) s.splinePoints.back().pos.y = it->second;
                    if (!s.controlPoints.empty()) s.controlPoints.back().y = it->second;
                }
            }
        }

        // Solve elevations for active chunk
        if (hasTerrain) {
            RoadElevationSolver::solve(curChunkSegs, sampleTerrain, solverOpts);
        }

        // Update solved boundary positions and register in seamSolvedElevations
        for (auto& s : curChunkSegs) {
            if (s.splinePoints.empty()) continue;
            if (s.startNeighbor.hasNeighbor) {
                float y = s.splinePoints.front().pos.y;
                s.startNeighbor.boundaryPos.y = y;
                seamSolvedElevations[makeSeamKey(s.startNeighbor.boundaryPos.x, s.startNeighbor.boundaryPos.z)] = y;
            }
            if (s.endNeighbor.hasNeighbor) {
                float y = s.splinePoints.back().pos.y;
                s.endNeighbor.boundaryPos.y = y;
                seamSolvedElevations[makeSeamKey(s.endNeighbor.boundaryPos.x, s.endNeighbor.boundaryPos.z)] = y;
            }
        }

        // Build network topology & junctions for this chunk
        RoadGenerator::collapseShortJunctionSegments(curChunkSegs, 26.0f);
        std::vector<RoadJunction> junctions;
        RoadGenerator::buildNetworkTopology(curChunkSegs, junctions);
        RoadGenerator::generateJunctionPatches(curChunkSegs, junctions);

        // Save chunk binary (.froad)
        float minX = cx * options.chunkSize - halfS;
        float maxX = cx * options.chunkSize + halfS;
        float minZ = cz * options.chunkSize - halfS;
        float maxZ = cz * options.chunkSize + halfS;

        std::string chunkFilename = options.outputDir + "/chunk_" + std::to_string(cx) + "_" + std::to_string(cz) + ".froad";
        if (!RoadBinary::saveChunk(chunkFilename, cx, cz, minX, maxX, minZ, maxZ, curChunkSegs, junctions)) {
            result.errorMessage = "Failed to write chunk binary: " + chunkFilename;
            if (options.verbose) std::cerr << "RoadBaker: " << result.errorMessage << std::endl;
            return result;
        }

        result.totalChunks++;
        result.totalSegments += static_cast<uint32_t>(curChunkSegs.size());
        for (const auto& s : curChunkSegs) {
            result.totalLengthMeters += s.length;
            if (s.startNeighbor.hasNeighbor) result.totalCrossChunkConnections++;
            if (s.endNeighbor.hasNeighbor) result.totalCrossChunkConnections++;
        }

        if (options.verbose && (result.totalChunks % 100 == 0 || result.totalChunks == chunkSegments.size())) {
            std::cout << "RoadBaker: Baked [" << result.totalChunks << "/" << chunkSegments.size()
                      << "] chunks (" << result.totalSegments << " segments, "
                      << (result.totalLengthMeters / 1000.0f) << " km)..." << std::endl;
        }
    }

    result.success = true;

    if (options.verbose) {
        std::cout << "========================================\n"
                  << "RoadBaker: Multi-Chunk Bake Completed!\n"
                  << "  Total Chunks:           " << result.totalChunks << "\n"
                  << "  Total Segments:         " << result.totalSegments << "\n"
                  << "  Cross-Chunk Connections: " << result.totalCrossChunkConnections << "\n"
                  << "  Total Road Length:      " << (result.totalLengthMeters / 1000.0f) << " km\n"
                  << "  Output Directory:       " << options.outputDir << "\n"
                  << "========================================" << std::endl;
    }

    return result;
}
