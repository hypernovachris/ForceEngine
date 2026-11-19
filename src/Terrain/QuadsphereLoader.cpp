#include "Terrain/QuadsphereLoader.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <zstd.h>

bool QuadsphereLoader::openPack(const std::string& filepath, LoadedQuadspherePack& outPack) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "QuadsphereLoader: Failed to open pack file " << filepath << std::endl;
        return false;
    }

    outPack.filePath = filepath;

    // 1. Read 192-byte Pack Header
    file.read(reinterpret_cast<char*>(&outPack.header), sizeof(FQuadspherePackHeader));
    if (!file || !outPack.isValid()) {
        std::cerr << "QuadsphereLoader: Invalid or corrupted .fquadspherepack header in " << filepath << std::endl;
        return false;
    }

    // 2. Read Morton Tile Index Table
    file.seekg(outPack.header.indexOffset, std::ios::beg);
    outPack.indexTable.resize(outPack.header.tileCount);
    file.read(reinterpret_cast<char*>(outPack.indexTable.data()),
              outPack.header.tileCount * sizeof(QuadsphereTileIndexEntry));

    if (!file) {
        std::cerr << "QuadsphereLoader: Truncated index table in " << filepath << std::endl;
        return false;
    }

    return true;
}

const QuadsphereTileIndexEntry* QuadsphereLoader::findTileIndex(const LoadedQuadspherePack& pack,
                                                               uint8_t face, uint16_t lod,
                                                               uint32_t tx, uint32_t ty) {
    if (pack.indexTable.empty()) return nullptr;
    uint64_t targetKey = Quadsphere::packTileKey(face, lod, tx, ty);

    auto it = std::lower_bound(pack.indexTable.begin(), pack.indexTable.end(), targetKey,
                               [](const QuadsphereTileIndexEntry& entry, uint64_t key) {
                                   return entry.key < key;
                               });

    if (it != pack.indexTable.end() && it->key == targetKey) {
        return &(*it);
    }
    return nullptr;
}

bool QuadsphereLoader::decompressTilePayload(const uint8_t* compressedData, size_t compressedSize,
                                             int32_t quantizedMinZ,
                                             std::vector<uint16_t>& outHeights) {
    const int dim = 129;
    const size_t numSamples = dim * dim;
    const size_t uncompressedBytes = numSamples * 2;

    std::vector<uint8_t> shuffled(uncompressedBytes);
    size_t dSize = ZSTD_decompress(shuffled.data(), shuffled.size(), compressedData, compressedSize);
    if (ZSTD_isError(dSize) || dSize != uncompressedBytes) {
        std::cerr << "QuadsphereLoader: ZSTD_decompress failed: " << ZSTD_getErrorName(dSize) << std::endl;
        return false;
    }

    outHeights.resize(numSamples);

    // Unshuffle, Inverse ZigZag, and Inverse Paeth
    for (int y = 0; y < dim; ++y) {
        for (int x = 0; x < dim; ++x) {
            int idx = y * dim + x;
            uint16_t high = shuffled[idx];
            uint16_t low  = shuffled[numSamples + idx];
            uint32_t z = (high << 8) | low;
            int16_t res = static_cast<int16_t>((z >> 1) ^ (-(static_cast<int32_t>(z & 1))));

            uint16_t a = (x > 0) ? outHeights[y * dim + (x - 1)] : 0;
            uint16_t b = (y > 0) ? outHeights[(y - 1) * dim + x] : 0;
            uint16_t c = (x > 0 && y > 0) ? outHeights[(y - 1) * dim + (x - 1)] : 0;

            int32_t p = static_cast<int32_t>(a) + static_cast<int32_t>(b) - static_cast<int32_t>(c);
            int32_t pa = std::abs(p - static_cast<int32_t>(a));
            int32_t pb = std::abs(p - static_cast<int32_t>(b));
            int32_t pc = std::abs(p - static_cast<int32_t>(c));
            uint16_t pred = a;
            if (pa <= pb && pa <= pc) pred = a;
            else if (pb <= pc) pred = b;
            else pred = c;

            outHeights[idx] = static_cast<uint16_t>((static_cast<int32_t>(pred) + res) & 0xFFFF);
        }
    }
    return true;
}

bool QuadsphereLoader::compressTilePayload(const float* elevations,
                                           std::vector<uint8_t>& outCompressed,
                                           int32_t& outQuantizedMinZ) {
    const int dim = 129;
    const size_t numSamples = dim * dim;

    // 1. 5mm integer quantization
    int32_t hMin = 2147483647;
    std::vector<int32_t> H_global(numSamples);
    for (size_t i = 0; i < numSamples; ++i) {
        H_global[i] = static_cast<int32_t>(std::round(elevations[i] / 0.005f));
        hMin = std::min(hMin, H_global[i]);
    }
    outQuantizedMinZ = hMin;

    std::vector<uint16_t> q(numSamples);
    for (size_t i = 0; i < numSamples; ++i) {
        int32_t diff = H_global[i] - hMin;
        q[i] = static_cast<uint16_t>(std::clamp(diff, 0, 65535));
    }

    // 2. 2D Paeth Predictor
    std::vector<int16_t> residuals(numSamples);
    for (int y = 0; y < dim; ++y) {
        for (int x = 0; x < dim; ++x) {
            uint16_t a = (x > 0) ? q[y * dim + (x - 1)] : 0;
            uint16_t b = (y > 0) ? q[(y - 1) * dim + x] : 0;
            uint16_t c = (x > 0 && y > 0) ? q[(y - 1) * dim + (x - 1)] : 0;

            int32_t p = static_cast<int32_t>(a) + static_cast<int32_t>(b) - static_cast<int32_t>(c);
            int32_t pa = std::abs(p - static_cast<int32_t>(a));
            int32_t pb = std::abs(p - static_cast<int32_t>(b));
            int32_t pc = std::abs(p - static_cast<int32_t>(c));
            uint16_t pred = a;
            if (pa <= pb && pa <= pc) pred = a;
            else if (pb <= pc) pred = b;
            else pred = c;

            residuals[y * dim + x] = static_cast<int16_t>(static_cast<int32_t>(q[y * dim + x]) - pred);
        }
    }

    // 3. ZigZag + Byte Shuffle
    std::vector<uint8_t> shuffled(numSamples * 2);
    for (size_t i = 0; i < numSamples; ++i) {
        int32_t v = residuals[i];
        uint16_t zz = static_cast<uint16_t>((v << 1) ^ (v >> 15));
        shuffled[i] = static_cast<uint8_t>(zz >> 8);
        shuffled[numSamples + i] = static_cast<uint8_t>(zz & 0xFF);
    }

    // 4. Zstd Compress (level 9)
    outCompressed.resize(ZSTD_compressBound(shuffled.size()));
    size_t cSize = ZSTD_compress(outCompressed.data(), outCompressed.size(),
                                 shuffled.data(), shuffled.size(), 9);
    if (ZSTD_isError(cSize)) {
        std::cerr << "QuadsphereLoader: ZSTD_compress failed: " << ZSTD_getErrorName(cSize) << std::endl;
        return false;
    }
    outCompressed.resize(cSize);
    return true;
}

bool QuadsphereLoader::loadTile(const LoadedQuadspherePack& pack,
                                uint8_t face, uint16_t lod,
                                uint32_t tx, uint32_t ty,
                                DecodedQuadsphereTile& outTile) {
    const auto* entry = findTileIndex(pack, face, lod, tx, ty);
    if (!entry) return false;

    std::ifstream file(pack.filePath, std::ios::binary);
    if (!file.is_open()) return false;

    file.seekg(entry->byteOffset, std::ios::beg);
    std::vector<uint8_t> compData(entry->compressedSize);
    file.read(reinterpret_cast<char*>(compData.data()), entry->compressedSize);
    if (!file) return false;

    outTile.info = *entry;
    outTile.face = face;
    outTile.lod = lod;
    outTile.tx = tx;
    outTile.ty = ty;
    outTile.tileMinZ = entry->quantizedMinZ * 0.005f;

    return decompressTilePayload(compData.data(), compData.size(), entry->quantizedMinZ, outTile.heights);
}

bool QuadsphereLoader::hasElevation(const LoadedQuadspherePack& pack,
                                    double latDeg, double lonDeg,
                                    float& outElevation) {
    if (!pack.isValid() || pack.indexTable.empty()) return false;

    uint8_t face; double u, v;
    Quadsphere::geodeticToQuadsphere(latDeg, lonDeg, face, u, v);

    // Search pyramid from finest LOD to coarsest
    for (int lod = pack.header.maxLOD; lod >= pack.header.minLOD; --lod) {
        double tilesPerSide = static_cast<double>(1ULL << lod);
        double span = 2.0 / tilesPerSide;

        int tx = static_cast<int>(std::floor((u + 1.0) / span));
        int ty = static_cast<int>(std::floor((v + 1.0) / span));
        if (tx < 0 || ty < 0 || tx >= (int)tilesPerSide || ty >= (int)tilesPerSide) continue;

        const auto* entry = findTileIndex(pack, face, lod, tx, ty);
        if (!entry) continue;

        DecodedQuadsphereTile tile;
        if (loadTile(pack, face, lod, tx, ty, tile)) {
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
            outElevation = h0 * (1.0f - ry) + h1 * ry;
            return true;
        }
    }
    return false;
}

float QuadsphereLoader::sampleElevation(const LoadedQuadspherePack& pack,
                                        double latDeg, double lonDeg) {
    float el = 0.0f;
    if (hasElevation(pack, latDeg, lonDeg, el)) {
        return el;
    }
    return 0.0f;
}

void QuadsphereLoader::evaluateControlPatch(uint8_t face, uint16_t lod, uint32_t tx, uint32_t ty,
                                            const glm::dvec3& camPosECEF,
                                            QuadsphereTerrainUniforms& outUniforms) {
    double minU, maxU, minV, maxV;
    Quadsphere::getTileUVBounds(lod, tx, ty, minU, maxU, minV, maxV);

    for (int n = 0; n < 3; ++n) {
        double beta = static_cast<double>(n) * 0.5;
        double v = minV + beta * (maxV - minV);

        for (int m = 0; m < 3; ++m) {
            double alpha = static_cast<double>(m) * 0.5;
            double u = minU + alpha * (maxU - minU);

            glm::dvec3 P_sphere = Quadsphere::quadsphereToECEF(face, u, v, 0.0);
            glm::dvec3 n_up = Quadsphere::quadsphereToNormal(face, u, v);

            int idx = n * 3 + m;
            outUniforms.v_patch[idx] = glm::vec4(glm::vec3(P_sphere - camPosECEF), 0.0f);
            outUniforms.n_patch[idx] = glm::vec4(glm::vec3(n_up), 0.0f);
        }
    }
}
