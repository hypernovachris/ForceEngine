#ifndef QUADSPHERE_TYPES_H
#define QUADSPHERE_TYPES_H

#include <cstdint>
#include <vector>
#include <algorithm>
#include <glm/glm/glm.hpp>
#include "Terrain/QuadsphereMath.h"

#pragma pack(push, 1)

// Quadsphere Pack Header (192 bytes)
struct FQuadspherePackHeader {
    char     magic[4];           // 0..3: "QSPK"
    uint32_t version;            // 4..7: 1
    uint32_t tileCount;          // 8..11: Total tiles across all LODs in this container
    uint16_t minLOD;             // 12..13: Min LOD (e.g. 10)
    uint16_t maxLOD;             // 14..15: Max LOD (e.g. 18)
    uint64_t indexOffset;        // 16..23: Byte offset to start of TileIndexEntry table
    uint64_t dataOffset;         // 24..31: Byte offset to start of Zstd payload blob
    float    quantizationStep;   // 32..35: 0.005f (5mm per integer unit)
    uint32_t samplesPerTile;     // 36..39: 129
    double   earthRadius;        // 40..47: 6371000.0
    uint8_t  reserved[144];      // 48..191: Padding to 192 bytes
};

// Quadsphere Tile Index Entry (24 bytes) (SPHERE.md Section 4.2)
struct QuadsphereTileIndexEntry {
    uint64_t key;            // 8 bytes: [60..58]: face, [57..52]: lod, [51..0]: morton2D(tx, ty)
    uint64_t byteOffset;     // 8 bytes: Offset into pack file data blob
    uint32_t compressedSize; // 4 bytes: Zstd payload size in bytes
    int32_t  quantizedMinZ;  // 4 bytes: H_min in 5mm integer units
};

#pragma pack(pop)

// Static 131x131 Grid Vertex (SPHERE.md Section 3.1)
struct QuadsphereGridVertex {
    float uv[2];    // alpha, beta in [0, 1]
    float grid[2];  // integer sample coordinates i, j in [0, 128]
    float isSkirt;  // 0.0 for surface, 1.0 for perimeter skirt drop
};

// 3x3 Control Patch Uniforms for Relative-To-Eye (RTE) Terrain Shader (SPHERE.md Section 3.2)
struct QuadsphereTerrainUniforms {
    glm::mat4 projection;   // Standard perspective projection
    glm::mat4 viewRot;      // View rotation (translation = 0 for RTE)
    glm::vec4 v_patch[9];   // 3x3 control points (P_sphere - C_ECEF, w=0)
    glm::vec4 n_patch[9];   // 3x3 control normals (n_up, w=0)
    glm::vec4 tileParams;   // x: tileMinZ, y: skirtDepth, z: lod, w: lodColorTint
    glm::vec4 sunDir;       // Directional sun light vector
};

// Decoded Tile in Memory
struct DecodedQuadsphereTile {
    QuadsphereTileIndexEntry info;
    uint8_t  face = 0;
    uint16_t lod = 0;
    uint32_t tx = 0;
    uint32_t ty = 0;
    float    tileMinZ = 0.0f;
    std::vector<uint16_t> heights; // 129 x 129 = 16,641 samples (quantized uint16)

    inline float getElevation(int x, int y, int dim = 129) const {
        if (x < 0) x = 0; if (x >= dim) x = dim - 1;
        if (y < 0) y = 0; if (y >= dim) y = dim - 1;
        if (heights.empty()) return tileMinZ;
        return tileMinZ + static_cast<float>(heights[y * dim + x]) * 0.005f;
    }
};

#endif // QUADSPHERE_TYPES_H
