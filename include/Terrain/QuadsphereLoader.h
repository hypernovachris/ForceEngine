#ifndef QUADSPHERE_LOADER_H
#define QUADSPHERE_LOADER_H

#include <string>
#include <vector>
#include "Terrain/QuadsphereTypes.h"

struct LoadedQuadspherePack {
    std::string filePath;
    FQuadspherePackHeader header;
    std::vector<QuadsphereTileIndexEntry> indexTable;

    bool isValid() const {
        return header.magic[0] == 'Q' && header.magic[1] == 'S' &&
               header.magic[2] == 'P' && header.magic[3] == 'K';
    }
};

class QuadsphereLoader {
public:
    // Open .fquadspherepack container and read Morton index table
    static bool openPack(const std::string& filepath, LoadedQuadspherePack& outPack);

    // O(log N) binary search for tile index entry
    static const QuadsphereTileIndexEntry* findTileIndex(const LoadedQuadspherePack& pack,
                                                         uint8_t face, uint16_t lod,
                                                         uint32_t tx, uint32_t ty);

    // Decompress tile from pack via Zstd + inverse Byte-Shuffle + inverse ZigZag + inverse Paeth
    static bool loadTile(const LoadedQuadspherePack& pack,
                         uint8_t face, uint16_t lod,
                         uint32_t tx, uint32_t ty,
                         DecodedQuadsphereTile& outTile);

    // Sample terrain elevation at given WGS84 Geodetic coordinates (Lat, Lon degrees)
    static bool hasElevation(const LoadedQuadspherePack& pack,
                             double latDeg, double lonDeg,
                             float& outElevation);

    static float sampleElevation(const LoadedQuadspherePack& pack,
                                double latDeg, double lonDeg);

    // Evaluate 3x3 Control Patch for Relative-To-Eye (RTE) shader (SPHERE.md Section 3.2)
    static void evaluateControlPatch(uint8_t face, uint16_t lod, uint32_t tx, uint32_t ty,
                                     const glm::dvec3& camPosECEF,
                                     QuadsphereTerrainUniforms& outUniforms);

    // Decompress in-memory Zstd payload blob into 129x129 quantized uint16 heights
    static bool decompressTilePayload(const uint8_t* compressedData, size_t compressedSize,
                                      int32_t quantizedMinZ,
                                      std::vector<uint16_t>& outHeights);

    // Compress 129x129 float elevations into Zstd payload blob
    static bool compressTilePayload(const float* elevations,
                                    std::vector<uint8_t>& outCompressed,
                                    int32_t& outQuantizedMinZ);
};

#endif // QUADSPHERE_LOADER_H
