#ifndef ROAD_BAKER_H
#define ROAD_BAKER_H

#include <string>
#include <vector>
#include <cstdint>
#include "Road/RoadTypes.h"

struct RoadBakeOptions {
    std::string geoJsonPath = "jughandle/export.geojson";
    std::string terrainPackPath = "jughandle/somerset_mc_lerc.fterrainpack";
    std::string outputPath = "assets/maps/jughandle_500m.froad";
    bool drapeOnTerrain = true;
    bool filter500mOnly = true;
    float groundOffset = 0.15f;
    float bridgeClearance = 5.5f;
    bool verbose = true;
};

struct RoadBakeResult {
    bool success = false;
    uint32_t rawFeatureCount = 0;
    uint32_t processedSegmentCount = 0;
    uint32_t junctionCount = 0;
    uint32_t totalVertices = 0;
    uint32_t totalTriangles = 0;
    float totalLengthMeters = 0.0f;
    size_t outputFileSize = 0;
    std::string errorMessage;
};

struct MultiChunkBakeOptions {
    std::string geoJsonPath = "jughandle/new_jersey_roads.geojson";
    std::string terrainPackPath = "jughandle/new_jersey_05m.fterrainpack";
    std::string outputDir = "assets/maps/new_jersey_chunks";
    float chunkSize = 512.0f;
    float haloMargin = 96.0f;
    float groundOffset = 0.15f;
    float bridgeClearance = 5.5f;
    bool verbose = true;
};

struct MultiChunkBakeResult {
    bool success = false;
    uint32_t totalChunks = 0;
    uint32_t totalSegments = 0;
    uint32_t totalCrossChunkConnections = 0;
    uint32_t totalVertices = 0;
    uint32_t totalTriangles = 0;
    float totalLengthMeters = 0.0f;
    std::string errorMessage;
};

class RoadBaker {
public:
    // Standalone pipeline to bake raw GeoJSON (and optional LiDAR terrain) into single .froad binary
    static RoadBakeResult bake(const RoadBakeOptions& options);

    // Multi-chunk domain decomposition pipeline: splits road network across 512x512m grid,
    // connects cross-chunk segments, solves elevations with Dirichlet boundary conditions, and saves .froad chunks
    static MultiChunkBakeResult bakeChunks(const MultiChunkBakeOptions& options);
};

#endif // ROAD_BAKER_H

