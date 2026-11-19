#include "Terrain/TerrainLoader.h"
#include "Lerc_c_api.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <zlib.h>

uint64_t TerrainLoader::morton2D(uint32_t x, uint32_t y) {
    auto part1by1 = [](uint32_t n) -> uint64_t {
        uint64_t val = static_cast<uint64_t>(n) & 0x0000FFFFULL;
        val = (val | (val << 8)) & 0x00FF00FFULL;
        val = (val | (val << 4)) & 0x0F0F0F0FULL;
        val = (val | (val << 2)) & 0x33333333ULL;
        val = (val | (val << 1)) & 0x55555555ULL;
        return val;
    };
    return (part1by1(y) << 1) | part1by1(x);
}

uint64_t TerrainLoader::packTileKey(uint16_t lod, uint32_t tileX, uint32_t tileY) {
    return (static_cast<uint64_t>(lod) << 56) | (morton2D(tileX, tileY) & 0x00FFFFFFFFFFFFFFULL);
}

bool TerrainLoader::openPack(const std::string& filepath, LoadedTerrainPack& outPack) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "TerrainLoader: Failed to open pack file " << filepath << std::endl;
        return false;
    }

    outPack.filePath = filepath;

    // 1. Read 192-byte Pack Header
    file.read(reinterpret_cast<char*>(&outPack.header), sizeof(FTerrainPackHeader));
    if (!file || !outPack.isValid()) {
        std::cerr << "TerrainLoader: Invalid or corrupted .fterrainpack header in " << filepath << std::endl;
        return false;
    }

    // 2. Read Morton Tile Index Table
    file.seekg(outPack.header.indexOffset, std::ios::beg);
    outPack.indexTable.resize(outPack.header.tileCount);
    file.read(reinterpret_cast<char*>(outPack.indexTable.data()),
              outPack.header.tileCount * sizeof(FTerrainTileIndexEntry));

    if (!file) {
        std::cerr << "TerrainLoader: Truncated index table in " << filepath << std::endl;
        return false;
    }

    // Compute grid dimensions at LOD 0 and origin offsets
    if (outPack.header.reserved0 > 0 && outPack.header.reserved1 > 0) {
        outPack.tilesX_LOD0 = outPack.header.reserved0;
        outPack.tilesY_LOD0 = outPack.header.reserved1;
    } else {
        uint32_t maxTx = 0, maxTy = 0;
        for (const auto& entry : outPack.indexTable) {
            if (entry.lod == 0) {
                maxTx = std::max(maxTx, entry.tileX);
                maxTy = std::max(maxTy, entry.tileY);
            }
        }
        outPack.tilesX_LOD0 = maxTx + 1;
        outPack.tilesY_LOD0 = maxTy + 1;
    }
    outPack.startEast = - (outPack.tilesX_LOD0 * outPack.header.baseTileSize) * 0.5f;
    outPack.startNorth = - (outPack.tilesY_LOD0 * outPack.header.baseTileSize) * 0.5f;

    return true;
}

const FTerrainTileIndexEntry* TerrainLoader::findTileIndex(const LoadedTerrainPack& pack,
                                                          uint16_t lod,
                                                          uint32_t tileX,
                                                          uint32_t tileY) {
    if (pack.indexTable.empty()) return nullptr;
    uint64_t targetKey = packTileKey(lod, tileX, tileY);

    auto it = std::lower_bound(pack.indexTable.begin(), pack.indexTable.end(), targetKey,
        [](const FTerrainTileIndexEntry& entry, uint64_t key) {
            return entry.tileKey < key;
        });

    if (it != pack.indexTable.end() && it->tileKey == targetKey) {
        return &(*it);
    }
    return nullptr;
}

bool TerrainLoader::loadTile(const LoadedTerrainPack& pack,
                            uint16_t lod,
                            uint32_t tileX,
                            uint32_t tileY,
                            DecodedTerrainTile& outTile) {
    const FTerrainTileIndexEntry* entry = findTileIndex(pack, lod, tileX, tileY);
    if (!entry) return false;

    std::ifstream file(pack.filePath, std::ios::binary);
    if (!file.is_open()) return false;

    // Read compressed payload
    file.seekg(entry->byteOffset, std::ios::beg);
    std::vector<uint8_t> compressedData(entry->compressedSize);
    file.read(reinterpret_cast<char*>(compressedData.data()), entry->compressedSize);
    if (!file) return false;

    outTile.info = *entry;
    uint32_t sampleCount = pack.header.samplesPerTile * pack.header.samplesPerTile;
    outTile.elevations.resize(sampleCount);

    if (entry->format == 2) {
        // LERC float32 compression
        lerc_status status = lerc_decode(
            compressedData.data(),
            static_cast<unsigned int>(entry->compressedSize),
            0,          // nMasks
            nullptr,    // pValidBytes
            1,          // nDepth
            static_cast<int>(pack.header.samplesPerTile), // nCols (129)
            static_cast<int>(pack.header.samplesPerTile), // nRows (129)
            1,          // nBands
            6,          // dataType = 6 (float32)
            outTile.elevations.data()
        );
        if (status != 0) {
            std::cerr << "TerrainLoader: LERC float32 decode error " << status
                      << " for tile (" << lod << ", " << tileX << ", " << tileY << ")" << std::endl;
            return false;
        }
        return true;
    } else if (entry->format == 3) {
        // LERC uint16 normalized compression
        std::vector<uint16_t> raw16(sampleCount);
        lerc_status status = lerc_decode(
            compressedData.data(),
            static_cast<unsigned int>(entry->compressedSize),
            0,          // nMasks
            nullptr,    // pValidBytes
            1,          // nDepth
            static_cast<int>(pack.header.samplesPerTile), // nCols (129)
            static_cast<int>(pack.header.samplesPerTile), // nRows (129)
            1,          // nBands
            3,          // dataType = 3 (uint16)
            raw16.data()
        );
        if (status != 0) {
            std::cerr << "TerrainLoader: LERC uint16 decode error " << status
                      << " for tile (" << lod << ", " << tileX << ", " << tileY << ")" << std::endl;
            return false;
        }
        float range = entry->maxElevation - entry->minElevation;
        for (uint32_t i = 0; i < sampleCount; ++i) {
            float norm = static_cast<float>(raw16[i]) / 65535.0f;
            outTile.elevations[i] = entry->minElevation + norm * range;
        }
        return true;
    }

    // Decompress payload via Deflate
    std::vector<uint8_t> uncompressedData(entry->uncompressedSize);
    z_stream strm = {};
    strm.next_in = reinterpret_cast<Bytef*>(compressedData.data());
    strm.avail_in = static_cast<uInt>(entry->compressedSize);
    strm.next_out = reinterpret_cast<Bytef*>(uncompressedData.data());
    strm.avail_out = static_cast<uInt>(entry->uncompressedSize);

    if (inflateInit(&strm) != Z_OK) return false;
    int ret = inflate(&strm, Z_FINISH);
    inflateEnd(&strm);

    if (ret != Z_STREAM_END && ret != Z_OK) {
        std::cerr << "TerrainLoader: Deflate decompression error for tile (" << lod << ", " << tileX << ", " << tileY << ")" << std::endl;
        return false;
    }

    if (entry->format == 0) {
        // 16-bit normalized height
        const uint16_t* raw16 = reinterpret_cast<const uint16_t*>(uncompressedData.data());
        float range = entry->maxElevation - entry->minElevation;
        for (uint32_t i = 0; i < sampleCount; ++i) {
            float norm = static_cast<float>(raw16[i]) / 65535.0f;
            outTile.elevations[i] = entry->minElevation + norm * range;
        }
    } else {
        // 32-bit float height
        const float* raw32 = reinterpret_cast<const float*>(uncompressedData.data());
        for (uint32_t i = 0; i < sampleCount; ++i) {
            outTile.elevations[i] = raw32[i];
        }
    }

    return true;
}

float TerrainLoader::sampleTileHeight(const DecodedTerrainTile& tile,
                                      float localX,
                                      float localZ,
                                      float tileSize) {
    if (tile.elevations.empty()) return tile.info.minElevation;
    const int dim = 129;

    float u = std::clamp(localX / tileSize, 0.0f, 1.0f);
    float v = std::clamp(localZ / tileSize, 0.0f, 1.0f);

    float fx = u * static_cast<float>(dim - 1);
    float fz = v * static_cast<float>(dim - 1);

    int x0 = static_cast<int>(std::floor(fx));
    int z0 = static_cast<int>(std::floor(fz));
    int x1 = std::min(x0 + 1, dim - 1);
    int z1 = std::min(z0 + 1, dim - 1);

    float tx = fx - static_cast<float>(x0);
    float tz = fz - static_cast<float>(z0);

    float h00 = tile.elevations[z0 * dim + x0];
    float h10 = tile.elevations[z0 * dim + x1];
    float h01 = tile.elevations[z1 * dim + x0];
    float h11 = tile.elevations[z1 * dim + x1];

    float h0 = glm::mix(h00, h10, tx);
    float h1 = glm::mix(h01, h11, tx);
    return glm::mix(h0, h1, tz);
}

float TerrainLoader::calculateEarthCurvatureDrop(float distanceMeters, float earthRadius) {
    return (distanceMeters * distanceMeters) / (2.0f * earthRadius);
}

glm::dvec3 TerrainLoader::geodeticToECEF(double latDeg, double lonDeg, double heightMeters) {
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = 2.0 * f - f * f;

    double phi = glm::radians(latDeg);
    double lam = glm::radians(lonDeg);
    double sinPhi = std::sin(phi);
    double cosPhi = std::cos(phi);
    double sinLam = std::sin(lam);
    double cosLam = std::cos(lam);

    double N = a / std::sqrt(1.0 - e2 * sinPhi * sinPhi);

    double x = (N + heightMeters) * cosPhi * cosLam;
    double y = (N + heightMeters) * cosPhi * sinLam;
    double z = (N * (1.0 - e2) + heightMeters) * sinPhi;
    return glm::dvec3(x, y, z);
}

glm::vec3 TerrainLoader::ecefToENU(const glm::dvec3& ecefPoint, const glm::dvec3& ecefOrigin, const float* enuRotation3x3) {
    glm::dvec3 d = ecefPoint - ecefOrigin;
    float e = enuRotation3x3[0] * static_cast<float>(d.x) + enuRotation3x3[1] * static_cast<float>(d.y) + enuRotation3x3[2] * static_cast<float>(d.z);
    float n = enuRotation3x3[3] * static_cast<float>(d.x) + enuRotation3x3[4] * static_cast<float>(d.y) + enuRotation3x3[5] * static_cast<float>(d.z);
    float u = enuRotation3x3[6] * static_cast<float>(d.x) + enuRotation3x3[7] * static_cast<float>(d.y) + enuRotation3x3[8] * static_cast<float>(d.z);
    return glm::vec3(e, n, u);
}

glm::dvec3 TerrainLoader::enuToECEF(const glm::vec3& enuPoint, const glm::dvec3& ecefOrigin, const float* enuRotation3x3) {
    // Inverse of orthonormal rotation matrix R_enu is its transpose R_enu^T
    // enuPoint.x = East, enuPoint.y = North, enuPoint.z = Up
    double dx = enuRotation3x3[0] * enuPoint.x + enuRotation3x3[3] * enuPoint.y + enuRotation3x3[6] * enuPoint.z;
    double dy = enuRotation3x3[1] * enuPoint.x + enuRotation3x3[4] * enuPoint.y + enuRotation3x3[7] * enuPoint.z;
    double dz = enuRotation3x3[2] * enuPoint.x + enuRotation3x3[5] * enuPoint.y + enuRotation3x3[8] * enuPoint.z;
    return ecefOrigin + glm::dvec3(dx, dy, dz);
}

glm::dvec3 TerrainLoader::ecefToGeodetic(const glm::dvec3& ecefPoint) {
    // Bowring's closed-form algorithm for WGS84 ECEF -> Geodetic (lat, lon, height)
    const double a = 6378137.0;            // Semi-major axis
    const double f = 1.0 / 298.257223563;  // Flattening
    const double b = a * (1.0 - f);        // Semi-minor axis (~6356752.314245)
    const double e2 = 2.0 * f - f * f;     // First eccentricity squared
    const double ep2 = (a * a - b * b) / (b * b); // Second eccentricity squared

    double x = ecefPoint.x;
    double y = ecefPoint.y;
    double z = ecefPoint.z;

    double p = std::sqrt(x * x + y * y);
    if (p < 1e-6) {
        double lat = (z > 0.0) ? 90.0 : -90.0;
        return glm::dvec3(lat, 0.0, std::abs(z) - b);
    }

    double theta = std::atan2(z * a, p * b);
    double sinT = std::sin(theta);
    double cosT = std::cos(theta);

    double phi = std::atan2(z + ep2 * b * (sinT * sinT * sinT),
                            p - e2 * a * (cosT * cosT * cosT));
    double lam = std::atan2(y, x);

    double sinPhi = std::sin(phi);
    double N = a / std::sqrt(1.0 - e2 * sinPhi * sinPhi);
    double h = p / std::cos(phi) - N;

    return glm::dvec3(glm::degrees(phi), glm::degrees(lam), h);
}

std::array<FrustumPlane, 6> TerrainLoader::extractFrustumPlanes(const glm::mat4& vp,
                                                               const glm::vec3& camPos,
                                                               const glm::vec3& camForward,
                                                               float nearZ,
                                                               float farZ) {
    std::array<FrustumPlane, 6> planes;
    // Left: row3 + row0
    planes[0].normal.x = vp[0][3] + vp[0][0];
    planes[0].normal.y = vp[1][3] + vp[1][0];
    planes[0].normal.z = vp[2][3] + vp[2][0];
    planes[0].distance = vp[3][3] + vp[3][0];

    // Right: row3 - row0
    planes[1].normal.x = vp[0][3] - vp[0][0];
    planes[1].normal.y = vp[1][3] - vp[1][0];
    planes[1].normal.z = vp[2][3] - vp[2][0];
    planes[1].distance = vp[3][3] - vp[3][0];

    // Bottom: row3 + row1
    planes[2].normal.x = vp[0][3] + vp[0][1];
    planes[2].normal.y = vp[1][3] + vp[1][1];
    planes[2].normal.z = vp[2][3] + vp[2][1];
    planes[2].distance = vp[3][3] + vp[3][1];

    // Top: row3 - row1
    planes[3].normal.x = vp[0][3] - vp[0][1];
    planes[3].normal.y = vp[1][3] - vp[1][1];
    planes[3].normal.z = vp[2][3] - vp[2][1];
    planes[3].distance = vp[3][3] - vp[3][1];

    // Normalize lateral frustum planes (0..3)
    for (int i = 0; i < 4; ++i) {
        float len = glm::length(planes[i].normal);
        if (len > 1e-6f) {
            planes[i].normal /= len;
            planes[i].distance /= len;
        }
    }

    // Near plane: analytic calculation along camera forward vector (avoids catastrophic float cancellation)
    glm::vec3 fwd = glm::length(camForward) > 1e-6f ? glm::normalize(camForward) : glm::vec3(0.0f, 0.0f, -1.0f);
    planes[4].normal = fwd;
    planes[4].distance = -glm::dot(fwd, camPos) - nearZ;

    // Far plane: analytic calculation along opposite camera forward vector
    planes[5].normal = -fwd;
    planes[5].distance = glm::dot(fwd, camPos) + farZ;

    return planes;
}

std::array<FrustumPlane, 6> TerrainLoader::extractFrustumPlanes(const glm::mat4& vp) {
    std::array<FrustumPlane, 6> planes;
    // Left: row3 + row0
    planes[0].normal.x = vp[0][3] + vp[0][0];
    planes[0].normal.y = vp[1][3] + vp[1][0];
    planes[0].normal.z = vp[2][3] + vp[2][0];
    planes[0].distance = vp[3][3] + vp[3][0];

    // Right: row3 - row0
    planes[1].normal.x = vp[0][3] - vp[0][0];
    planes[1].normal.y = vp[1][3] - vp[1][0];
    planes[1].normal.z = vp[2][3] - vp[2][0];
    planes[1].distance = vp[3][3] - vp[3][0];

    // Bottom: row3 + row1
    planes[2].normal.x = vp[0][3] + vp[0][1];
    planes[2].normal.y = vp[1][3] + vp[1][1];
    planes[2].normal.z = vp[2][3] + vp[2][1];
    planes[2].distance = vp[3][3] + vp[3][1];

    // Top: row3 - row1
    planes[3].normal.x = vp[0][3] - vp[0][1];
    planes[3].normal.y = vp[1][3] - vp[1][1];
    planes[3].normal.z = vp[2][3] - vp[2][1];
    planes[3].distance = vp[3][3] - vp[3][1];

    // Near: row2 (Metal / DirectX / Vulkan depth [0, 1] clip space)
    planes[4].normal.x = vp[0][2];
    planes[4].normal.y = vp[1][2];
    planes[4].normal.z = vp[2][2];
    planes[4].distance = vp[3][2];

    // Far: row3 - row2
    planes[5].normal.x = vp[0][3] - vp[0][2];
    planes[5].normal.y = vp[1][3] - vp[1][2];
    planes[5].normal.z = vp[2][3] - vp[2][2];
    planes[5].distance = vp[3][3] - vp[3][2];

    for (int i = 0; i < 6; ++i) {
        float len = glm::length(planes[i].normal);
        if (len > 1e-6f) {
            planes[i].normal /= len;
            planes[i].distance /= len;
        }
    }
    return planes;
}

bool TerrainLoader::isBoxInFrustum(const std::array<FrustumPlane, 6>& planes,
                                   const glm::vec3& boxMin, const glm::vec3& boxMax) {
    for (int i = 0; i < 6; ++i) {
        glm::vec3 p;
        p.x = (planes[i].normal.x >= 0.0f) ? boxMax.x : boxMin.x;
        p.y = (planes[i].normal.y >= 0.0f) ? boxMax.y : boxMin.y;
        p.z = (planes[i].normal.z >= 0.0f) ? boxMax.z : boxMin.z;
        if (glm::dot(planes[i].normal, p) + planes[i].distance < 0.0f) {
            return false;
        }
    }
    return true;
}

