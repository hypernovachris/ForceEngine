#ifndef TERRAIN_TYPES_H
#define TERRAIN_TYPES_H

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <glm/glm/glm.hpp>

#pragma pack(push, 1)

// Container Pack File Header: Magic "FTPK" (Force Terrain Pack)
struct FTerrainPackHeader {
    char     magic[4];           // 0..3 (4B): "FTPK"
    uint32_t version;            // 4..7 (4B): 2
    float    baseTileSize;       // 8..11 (4B): Base LOD 0 physical tile size in meters (e.g. 64.0f)
    uint32_t samplesPerTile;     // 12..15 (4B): Raster dimension per side (129 for 128 quads + 1 vertex)
    float    baseResolution;     // 16..19 (4B): Base sample spacing in meters (e.g. 0.5f)
    uint32_t minLOD;             // 20..23 (4B): Minimum LOD level (0)
    uint32_t maxLOD;             // 24..27 (4B): Maximum LOD level in pyramid (e.g. 6 or 7)
    uint32_t tileCount;          // 28..31 (4B): Total number of tiles across all LODs in this container
    uint32_t reserved0;          // 32..35 (4B)
    uint32_t reserved1;          // 36..39 (4B)
    uint64_t indexOffset;        // 40..47 (8B): Byte offset to start of Morton Tile Index Table
    uint64_t dataOffset;         // 48..55 (8B): Byte offset to start of compressed tile payload blob
    float    globalMinElevation; // 56..59 (4B): Global minimum elevation in meters
    float    globalMaxElevation; // 60..63 (4B): Global maximum elevation in meters
    double   ecefAnchor[3];      // 64..87 (24B): 64-bit ECEF (X, Y, Z) coordinates of domain origin (meters)
    double   geodeticAnchor[2];  // 88..103 (16B): WGS84 Latitude, Longitude of origin (degrees)
    float    enuRotation[9];     // 104..139 (36B): 3x3 orthonormal rotation matrix (ECEF -> ENU)
    float    earthRadius;        // 140..143 (4B): Mean Earth radius (6,371,000.0m)
    uint32_t flags;              // 144..147 (4B): Bit 0: IsUint16 (1) vs Float32 (0), Bit 1: Deflate (1)
    uint8_t  reserved[44];       // 148..191 (44B): Padding to clean 192-byte header
};

// Morton / Z-Order Tile Index Table Entry (44 bytes each)
struct FTerrainTileIndexEntry {
    uint64_t tileKey;            // 8 bytes: (uint64_t(lod) << 56) | morton2D(tileX, tileY)
    uint32_t tileX;              // 4 bytes: Tile X index at this LOD
    uint32_t tileY;              // 4 bytes: Tile Y index at this LOD
    uint16_t lod;                // 2 bytes: LOD level (0 = 64m, 1 = 128m, 3 = 512m road chunk...)
    uint16_t format;             // 2 bytes: 0 = uint16 normalized, 1 = float32
    uint64_t byteOffset;         // 8 bytes: File offset to compressed tile data blob
    uint32_t compressedSize;     // 4 bytes: Compressed payload bytes on disk
    uint32_t uncompressedSize;   // 4 bytes: Uncompressed size (129*129*2 = 33,282 bytes for uint16)
    float    minElevation;       // 4 bytes: Tile minimum elevation in meters
    float    maxElevation;       // 4 bytes: Tile maximum elevation in meters
};

#pragma pack(pop)

// Runtime Decompressed Tile Representation
struct DecodedTerrainTile {
    FTerrainTileIndexEntry info;
    std::vector<float> elevations; // Exactly samplesPerTile * samplesPerTile (129 * 129 = 16,641 floats)

    float getElevation(int x, int y, int dim = 129) const {
        if (x < 0) x = 0; if (x >= dim) x = dim - 1;
        if (y < 0) y = 0; if (y >= dim) y = dim - 1;
        return elevations[y * dim + x];
    }
};

// Runtime In-Engine Terrain Pack Handle
struct LoadedTerrainPack {
    FTerrainPackHeader header;
    std::vector<FTerrainTileIndexEntry> indexTable;
    std::string filePath;
    uint32_t tilesX_LOD0 = 0;
    uint32_t tilesY_LOD0 = 0;
    float startEast = 0.0f;
    float startNorth = 0.0f;

    bool isValid() const {
        return header.magic[0] == 'F' && header.magic[1] == 'T' &&
               header.magic[2] == 'P' && header.magic[3] == 'K';
    }
};

// Frustum plane representation for culling
struct FrustumPlane {
    glm::vec3 normal = glm::vec3(0.0f);
    float distance = 0.0f;
};

#endif // TERRAIN_TYPES_H
