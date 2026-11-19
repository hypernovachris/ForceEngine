#ifndef ROAD_BINARY_H
#define ROAD_BINARY_H

#include <string>
#include <vector>
#include <memory>
#include "Road/RoadTypes.h"
#include "Mesh.h"

class RoadBinary {
public:
    // Compile and export a Quadsphere Level 15 chunk to .froad (Version 3 with 64-bit ECEF RTE anchor)
    static bool saveQuadsphereChunk(const std::string& filepath,
                                   uint8_t face, uint16_t lod,
                                   uint32_t tileX, uint32_t tileY,
                                   const glm::dvec3& chunkCenterECEF,
                                   float minU, float maxU,
                                   float minV, float maxV,
                                   const std::vector<ProcessedRoadSegment>& segments,
                                   const std::vector<RoadJunction>& junctions = {});

    // Compile and export a 512m x 512m chunk to .froad (Version 2 with multi-chunk & cross-chunk topology)
    static bool saveChunk(const std::string& filepath,
                          int chunkX, int chunkZ,
                          float minX, float maxX,
                          float minZ, float maxZ,
                          const std::vector<ProcessedRoadSegment>& segments,
                          const std::vector<RoadJunction>& junctions = {});

    // Legacy 500m chunk export
    static bool save500mChunk(const std::string& filepath,
                              const std::vector<ProcessedRoadSegment>& segments,
                              const std::vector<RoadJunction>& junctions = {});

    // Load compiled .froad intermediate from disk (supports both Version 1 and Version 2)
    static bool loadChunk(const std::string& filepath,
                          std::vector<ProcessedRoadSegment>& outSegments,
                          std::shared_ptr<Mesh>& outMergedMesh,
                          FRoadChunkHeader& outHeader,
                          std::vector<RoadJunction>* outJunctions = nullptr);
};

#endif // ROAD_BINARY_H

