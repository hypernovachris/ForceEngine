#ifndef ROAD_GENERATOR_H
#define ROAD_GENERATOR_H

#include <vector>
#include <memory>
#include <map>
#include <glm/glm/glm.hpp>
#include "Road/RoadTypes.h"
#include "Mesh.h"

class RoadGenerator {
public:
    // Multi-chunk domain decomposition: Split continuous segments on 512m chunk boundaries,
    // establish inter-chunk topological neighbor links, and group by chunk grid (chunkX, chunkZ)
    static void splitSegmentsAcrossGrid(const std::vector<ProcessedRoadSegment>& inSegments,
                                       float chunkSize,
                                       std::map<std::pair<int, int>, std::vector<ProcessedRoadSegment>>& outChunkSegments);

    // Multi-chunk domain decomposition across Level 15 Quadsphere tiles (SPHERE.md Section 5.1)
    static void splitSegmentsAcrossQuadsphere(
        const std::vector<ProcessedRoadSegment>& inSegments,
        uint16_t lod, // Level 15
        std::map<std::tuple<uint8_t, uint32_t, uint32_t>, std::vector<ProcessedRoadSegment>>& outChunkSegments);

    // Generate uniform arc-length sampled spline points from raw GPS control points
    static std::vector<RoadSplinePoint> generateSpline(const std::vector<glm::vec3>& rawPoints,
                                                       float sampleStep = 2.5f,
                                                       const glm::dvec3* optChunkCenterECEF = nullptr);

    // Deduce road geometry parameters from raw feature properties using real tags first,
    // falling back to authoritative highway engineering standards (AASHTO Interstate / Arterial / Local)
    static void deduceRoadParameters(const RawRoadFeature& feat,
                                     float& outLaneWidth,
                                     float& outShoulderWidth,
                                     int& outLanes,
                                     bool& outOneWay,
                                     std::string& outWidthSource);

    // Generate ribbon vertices and indices for a single road segment, encoding lane markings
    static void generateRibbonGeometry(const std::vector<RoadSplinePoint>& splinePoints,
                                       float totalWidth,
                                       int lanes,
                                       bool oneway,
                                       float shoulderWidth,
                                       std::vector<Vertex>& outVertices,
                                       std::vector<uint32_t>& outIndices);

    // Backward-compatible overload
    static void generateRibbonGeometry(const std::vector<RoadSplinePoint>& splinePoints,
                                       float totalWidth,
                                       std::vector<Vertex>& outVertices,
                                       std::vector<uint32_t>& outIndices) {
        generateRibbonGeometry(splinePoints, totalWidth, 2, false, totalWidth * 0.1f, outVertices, outIndices);
    }

    // Process a collection of raw features into renderable segments
    static void sanitizeRoadFeatures(std::vector<RawRoadFeature>& features,
                                     float minControlPointDist = 6.0f,
                                     float minSegmentLength = 16.0f);

    static void absorbMicroSegments(std::vector<RawRoadFeature>& features,
                                    float minSegmentLength = 26.0f);

    static void stitchCollinearFeatures(std::vector<RawRoadFeature>& features);

    // Detect road intersections (T-intersections, interior node touches, and at-grade crossings)
    // where roads intersect but were not split in source data, and split them to create junctions.
    static void splitIntersectingFeatures(std::vector<RawRoadFeature>& features,
                                          float snapDistance = 6.0f,
                                          float minJunctionDistance = 22.0f);

    // Collapse any short segment squeezed between two multi-way junctions and merge them into one junction
    static bool collapseShortJunctionSegments(std::vector<ProcessedRoadSegment>& segments,
                                              float minLength = 26.0f);

    static void processFeatures(const std::vector<RawRoadFeature>& rawFeatures,
                                float laneWidth,
                                float shoulderWidth,
                                float sampleStep,
                                std::vector<ProcessedRoadSegment>& outSegments,
                                bool useOverrides = false,
                                bool generateVisualMeshes = true);

    // Build topological road network graph by identifying shared segment endpoints
    static void buildNetworkTopology(std::vector<ProcessedRoadSegment>& segments,
                                     std::vector<RoadJunction>& outJunctions,
                                     bool verbose = false);

    // Trim spline points back by startSetback and endSetback distances
    static std::vector<RoadSplinePoint> trimSpline(const std::vector<RoadSplinePoint>& spline,
                                                   float startSetback,
                                                   float endSetback);

    // Compute setbacks for segments meeting at multi-road junctions and generate planar junction polygon patches
    static void generateJunctionPatches(std::vector<ProcessedRoadSegment>& segments,
                                        std::vector<RoadJunction>& junctions,
                                        const glm::dvec3* optChunkCenterECEF = nullptr);

    // Clip ribbon vertices visually against neighboring acute road branches at junctions to guarantee zero Z-fighting
    static void clipRibbonAgainstJunctions(std::vector<Vertex>& vertices,
                                           const ProcessedRoadSegment& seg,
                                           const std::vector<RoadJunction>& junctions,
                                           const std::vector<ProcessedRoadSegment>& allSegments);

    // Merge multiple segments and junction patches into a single unified mesh for 1-draw-call rendering
    static std::shared_ptr<Mesh> buildMergedMesh(const std::vector<ProcessedRoadSegment>& segments,
                                                 const std::vector<RoadJunction>& junctions = {},
                                                 bool filterChunk = false,
                                                 float minX = -256.0f, float maxX = 256.0f,
                                                 float minZ = -256.0f, float maxZ = 256.0f);
};

#endif // ROAD_GENERATOR_H

