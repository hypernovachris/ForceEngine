#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include "Terrain/QuadsphereMath.h"
#include "Terrain/QuadsphereTypes.h"
#include "Terrain/QuadsphereLoader.h"
#include "Terrain/TerrainLoader.h"

int main(int argc, char* argv[]) {
    std::cout << "=== ForceEngine Quadsphere Terrain Compiler (SPHERE.md Phase 2) ===" << std::endl;

    std::string inputPackPath = "jughandle/somerset_dem.bin";
    std::string outputPackPath = "jughandle/somerset.fquadspherepack";

    if (argc > 1) inputPackPath = argv[1];
    if (argc > 2) outputPackPath = argv[2];

    std::cout << "Input Terrain Source: " << inputPackPath << std::endl;
    std::cout << "Output Quadsphere Pack: " << outputPackPath << std::endl;

    struct DemRaster {
        double minLon = 0, minLat = 0, maxLon = 0, maxLat = 0;
        uint32_t width = 0, height = 0;
        std::vector<float> data;
    } demRaster;

    bool hasDemRaster = false;
    LoadedTerrainPack srcPack;
    std::unordered_map<uint64_t, DecodedTerrainTile> sourceTileCache;
    glm::dvec3 ecefOrigin(0.0);
    const float* enuRot = nullptr;

    if (inputPackPath.rfind(".bin") != std::string::npos && std::ifstream(inputPackPath).good()) {
        std::ifstream f(inputPackPath, std::ios::binary);
        if (f.good()) {
            f.read(reinterpret_cast<char*>(&demRaster.minLon), sizeof(double));
            f.read(reinterpret_cast<char*>(&demRaster.minLat), sizeof(double));
            f.read(reinterpret_cast<char*>(&demRaster.maxLon), sizeof(double));
            f.read(reinterpret_cast<char*>(&demRaster.maxLat), sizeof(double));
            f.read(reinterpret_cast<char*>(&demRaster.width), sizeof(uint32_t));
            f.read(reinterpret_cast<char*>(&demRaster.height), sizeof(uint32_t));
            demRaster.data.resize(demRaster.width * demRaster.height);
            f.read(reinterpret_cast<char*>(demRaster.data.data()), demRaster.data.size() * sizeof(float));
            hasDemRaster = true;
            std::cout << "Loaded direct DEM raster: " << demRaster.width << "x" << demRaster.height
                      << ", Lon [" << demRaster.minLon << ", " << demRaster.maxLon << "]"
                      << ", Lat [" << demRaster.minLat << ", " << demRaster.maxLat << "]" << std::endl;
        }
    } else {
        if (!TerrainLoader::openPack(inputPackPath, srcPack)) {
            std::cerr << "Failed to open input terrain pack: " << inputPackPath << std::endl;
            return 1;
        }
        std::cout << "Loaded source pack (" << srcPack.header.tileCount << " tiles, LOD "
                  << srcPack.header.minLOD << ".." << srcPack.header.maxLOD << ")" << std::endl;
        ecefOrigin = glm::dvec3(srcPack.header.ecefAnchor[0], srcPack.header.ecefAnchor[1], srcPack.header.ecefAnchor[2]);
        enuRot = srcPack.header.enuRotation;
    }

    // Helper lambda to sample elevation at (lat, lon) from direct DEM or srcPack
    auto sampleSourceElevation = [&](double latDeg, double lonDeg) -> float {
        if (hasDemRaster) {
            double u = (lonDeg - demRaster.minLon) / (demRaster.maxLon - demRaster.minLon);
            double v = 1.0 - (latDeg - demRaster.minLat) / (demRaster.maxLat - demRaster.minLat);
            u = std::clamp(u, 0.0, 1.0);
            v = std::clamp(v, 0.0, 1.0);
            double fx = u * (demRaster.width - 1);
            double fy = v * (demRaster.height - 1);
            int ix = std::clamp(static_cast<int>(fx), 0, static_cast<int>(demRaster.width - 2));
            int iy = std::clamp(static_cast<int>(fy), 0, static_cast<int>(demRaster.height - 2));
            float rx = static_cast<float>(fx - ix);
            float ry = static_cast<float>(fy - iy);
            float h00 = demRaster.data[iy * demRaster.width + ix];
            float h10 = demRaster.data[iy * demRaster.width + (ix + 1)];
            float h01 = demRaster.data[(iy + 1) * demRaster.width + ix];
            float h11 = demRaster.data[(iy + 1) * demRaster.width + (ix + 1)];
            float h0 = h00 * (1.0f - rx) + h10 * rx;
            float h1 = h01 * (1.0f - rx) + h11 * rx;
            return h0 * (1.0f - ry) + h1 * ry;
        }

        glm::dvec3 ecef = TerrainLoader::geodeticToECEF(latDeg, lonDeg, 0.0);
        glm::vec3 enu = TerrainLoader::ecefToENU(ecef, ecefOrigin, enuRot);

        float posX = enu.x;
        float posZ = -enu.y;

        float relE = posX - srcPack.startEast;
        float relN = -posZ - srcPack.startNorth;

        float maxE = srcPack.tilesX_LOD0 * srcPack.header.baseTileSize;
        float maxN = srcPack.tilesY_LOD0 * srcPack.header.baseTileSize;

        if (relE < 0.0f || relN < 0.0f || relE > maxE || relN > maxN) {
            return 30.0f; // Default ground plane if out of bounds
        }

        for (int lod = 0; lod <= (int)srcPack.header.maxLOD; ++lod) {
            float tileSz = srcPack.header.baseTileSize * static_cast<float>(1 << lod);
            uint32_t tx = static_cast<uint32_t>(relE / tileSz);
            uint32_t ty = static_cast<uint32_t>(relN / tileSz);

            const auto* entry = TerrainLoader::findTileIndex(srcPack, lod, tx, ty);
            if (!entry) continue;

            uint64_t key = entry->tileKey;
            auto it = sourceTileCache.find(key);
            if (it == sourceTileCache.end()) {
                DecodedTerrainTile newTile;
                if (TerrainLoader::loadTile(srcPack, lod, tx, ty, newTile)) {
                    it = sourceTileCache.emplace(key, std::move(newTile)).first;
                } else {
                    continue;
                }
            }

            const DecodedTerrainTile& tile = it->second;
            float fracX = (relE - tx * tileSz) / tileSz;
            float fracZ = (relN - ty * tileSz) / tileSz;
            fracX = std::clamp(fracX, 0.0f, 1.0f);
            fracZ = std::clamp(fracZ, 0.0f, 1.0f);

            float fx = fracX * 128.0f;
            float fz = fracZ * 128.0f;
            int ix = std::clamp((int)fx, 0, 127);
            int iz = std::clamp((int)fz, 0, 127);
            float rx = fx - ix;
            float rz = fz - iz;

            float h00 = tile.elevations[iz * 129 + ix];
            float h10 = tile.elevations[iz * 129 + (ix + 1)];
            float h01 = tile.elevations[(iz + 1) * 129 + ix];
            float h11 = tile.elevations[(iz + 1) * 129 + (ix + 1)];

            float h0 = h00 * (1.0f - rx) + h10 * rx;
            float h1 = h01 * (1.0f - rx) + h11 * rx;
            return h0 * (1.0f - rz) + h1 * rz;
        }
        return 30.0f;
    };

    // Target Region: Somerset County including King George Rd & Katherine Dr (40.592676, -74.515211)
    double centerLat = 40.592676;
    double centerLon = -74.515211;

    uint8_t targetFace = 2; // Face 2 (-X face)
    double centerU, centerV;
    Quadsphere::geodeticToQuadsphere(centerLat, centerLon, targetFace, centerU, centerV);

    std::cout << "Target Center: Lat " << centerLat << ", Lon " << centerLon
              << " -> Face " << (int)targetFace << " (u=" << centerU << ", v=" << centerV << ")" << std::endl;

    // Radius in Quadsphere UV space (~8km x 8km domain centered on King George Rd & Katherine Dr)
    double halfSpanU = 0.00065;
    double halfSpanV = 0.00065;
    if (argc > 3) {
        double spanKm = std::stod(argv[3]);
        halfSpanU = (spanKm * 1000.0) / Quadsphere::EARTH_RADIUS;
        halfSpanV = halfSpanU;
    }
    double uMin = centerU - halfSpanU;
    double uMax = centerU + halfSpanU;
    double vMin = centerV - halfSpanV;
    double vMax = centerV + halfSpanV;

    const uint16_t minLOD = 10;
    const uint16_t maxLOD = 18;

    struct RawTilePayload {
        uint64_t key;
        int32_t quantizedMinZ;
        std::vector<uint8_t> compressedData;
    };

    std::vector<RawTilePayload> compiledTiles;
    auto startTime = std::chrono::high_resolution_clock::now();

    for (uint16_t lod = minLOD; lod <= maxLOD; ++lod) {
        double tilesPerSide = static_cast<double>(1ULL << lod);
        double span = 2.0 / tilesPerSide;

        int minTx = static_cast<int>(std::floor((uMin + 1.0) / span));
        int maxTx = static_cast<int>(std::floor((uMax + 1.0) / span));
        int minTy = static_cast<int>(std::floor((vMin + 1.0) / span));
        int maxTy = static_cast<int>(std::floor((vMax + 1.0) / span));

        minTx = std::clamp(minTx, 0, (int)tilesPerSide - 1);
        maxTx = std::clamp(maxTx, 0, (int)tilesPerSide - 1);
        minTy = std::clamp(minTy, 0, (int)tilesPerSide - 1);
        maxTy = std::clamp(maxTy, 0, (int)tilesPerSide - 1);

        int countThisLod = 0;
        for (int ty = minTy; ty <= maxTy; ++ty) {
            for (int tx = minTx; tx <= maxTx; ++tx) {
                double tileMinU, tileMaxU, tileMinV, tileMaxV;
                Quadsphere::getTileUVBounds(lod, tx, ty, tileMinU, tileMaxU, tileMinV, tileMaxV);

                std::vector<float> tileElevs(129 * 129);
                for (int j = 0; j < 129; ++j) {
                    double v = tileMinV + (static_cast<double>(j) / 128.0) * (tileMaxV - tileMinV);
                    for (int i = 0; i < 129; ++i) {
                        double u = tileMinU + (static_cast<double>(i) / 128.0) * (tileMaxU - tileMinU);

                        double sampleLat, sampleLon;
                        Quadsphere::quadsphereToGeodetic(targetFace, u, v, sampleLat, sampleLon);

                        float elev = sampleSourceElevation(sampleLat, sampleLon);
                        tileElevs[j * 129 + i] = elev;
                    }
                }

                RawTilePayload payload;
                payload.key = Quadsphere::packTileKey(targetFace, lod, tx, ty);
                if (QuadsphereLoader::compressTilePayload(tileElevs.data(), payload.compressedData, payload.quantizedMinZ)) {
                    compiledTiles.push_back(std::move(payload));
                    countThisLod++;
                }
            }
        }
        std::cout << "  LOD " << lod << " compiled: " << countThisLod << " tiles ("
                  << (maxTx - minTx + 1) << "x" << (maxTy - minTy + 1) << ")" << std::endl;
    }

    auto compEndTime = std::chrono::high_resolution_clock::now();
    double compSecs = std::chrono::duration<double>(compEndTime - startTime).count();
    std::cout << "Compressed " << compiledTiles.size() << " tiles in "
              << std::fixed << std::setprecision(2) << compSecs << " seconds ("
              << (compiledTiles.size() / compSecs) << " tiles/sec)" << std::endl;

    // Sort tiles strictly by 64-bit Morton key
    std::sort(compiledTiles.begin(), compiledTiles.end(), [](const RawTilePayload& a, const RawTilePayload& b) {
        return a.key < b.key;
    });

    // Build Pack File Layout
    uint64_t headerSize = sizeof(FQuadspherePackHeader); // 192 bytes
    uint64_t indexSize = compiledTiles.size() * sizeof(QuadsphereTileIndexEntry);
    uint64_t dataOffset = headerSize + indexSize;

    FQuadspherePackHeader header = {};
    header.magic[0] = 'Q'; header.magic[1] = 'S'; header.magic[2] = 'P'; header.magic[3] = 'K';
    header.version = 1;
    header.tileCount = static_cast<uint32_t>(compiledTiles.size());
    header.minLOD = minLOD;
    header.maxLOD = maxLOD;
    header.indexOffset = headerSize;
    header.dataOffset = dataOffset;
    header.quantizationStep = 0.005f; // 5mm
    header.samplesPerTile = 129;
    header.earthRadius = Quadsphere::EARTH_RADIUS;

    std::vector<QuadsphereTileIndexEntry> indexTable(compiledTiles.size());
    uint64_t currentPayloadOffset = dataOffset;
    uint64_t totalPayloadBytes = 0;

    for (size_t i = 0; i < compiledTiles.size(); ++i) {
        const auto& pt = compiledTiles[i];
        indexTable[i].key = pt.key;
        indexTable[i].byteOffset = currentPayloadOffset;
        indexTable[i].compressedSize = static_cast<uint32_t>(pt.compressedData.size());
        indexTable[i].quantizedMinZ = pt.quantizedMinZ;

        currentPayloadOffset += pt.compressedData.size();
        totalPayloadBytes += pt.compressedData.size();
    }

    // Write .fquadspherepack to disk
    std::ofstream outFile(outputPackPath, std::ios::binary);
    if (!outFile.is_open()) {
        std::cerr << "Failed to create output file: " << outputPackPath << std::endl;
        return 1;
    }

    outFile.write(reinterpret_cast<const char*>(&header), sizeof(header));
    outFile.write(reinterpret_cast<const char*>(indexTable.data()), indexTable.size() * sizeof(QuadsphereTileIndexEntry));
    for (const auto& pt : compiledTiles) {
        outFile.write(reinterpret_cast<const char*>(pt.compressedData.data()), pt.compressedData.size());
    }
    outFile.close();

    uint64_t totalFileBytes = dataOffset + totalPayloadBytes;
    double uncompBytes = static_cast<double>(compiledTiles.size()) * (129 * 129 * 4);
    std::cout << "Generated .fquadspherepack: " << outputPackPath << std::endl;
    std::cout << "  Total File Size: " << (totalFileBytes / (1024.0 * 1024.0)) << " MB" << std::endl;
    std::cout << "  Compression Ratio: " << (uncompBytes / totalPayloadBytes) << "x" << std::endl;
    std::cout << "  Tile Count: " << compiledTiles.size() << std::endl;

    return 0;
}
