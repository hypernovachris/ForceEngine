#ifndef ROAD_TYPES_H
#define ROAD_TYPES_H

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <glm/glm/glm.hpp>
#include "Mesh.h"

#pragma pack(push, 1)

enum class RoadClass : uint8_t {
    Trunk = 0,
    TrunkLink = 1,
    Secondary = 2,
    Tertiary = 3,
    Residential = 4
};

enum class LaneType : uint8_t {
    DrivableVehicle = 0,
    TurnPocketLeft = 1,
    TurnPocketRight = 2,
    ShoulderPaved = 3,
    ShoulderGravel = 4,
    Sidewalk = 5,
    MedianGrass = 6,
    BarrierJersey = 7
};

enum class MarkingType : uint8_t {
    None = 0,
    DashedWhite = 1,
    SolidWhite = 2,
    DoubleSolidYellow = 3,
    DashedYellow = 4
};

struct LaneElement {
    LaneType type;
    MarkingType leftMarking;
    float width; // meters
    float crossSlope;
    uint8_t speedLimitMph;
};

struct LaneLayoutPaletteEntry {
    uint8_t elementCount;
    LaneElement elements[8];
};

struct FRoadSamplePoint {
    float position[3]; // World-space XYZ
    float tangent[3];  // Forward travel direction unit vector
    float normal[3];   // Up vector
    float laneWidth;   // Lane width in meters
    uint32_t profileFlags;
};

struct FRoadSegmentRecord {
    uint32_t id;
    uint32_t splinePointOffset;
    uint16_t splinePointCount;
    uint8_t layoutProfileId;
    uint8_t flags; // Bit 0: Elevated/Bridge, Bit 1: Tunnel, Bit 2: OneWay
    uint32_t startJunctionId;
    uint32_t endJunctionId;
};

struct FRoadJunctionRecord {
    uint32_t id;
    float position[3];
    uint8_t connectionCount;
    uint8_t flags;
    uint16_t pad;
    uint32_t connectedSegmentIds[8];
};

// Cross-chunk topological connection record (Version 2+)
struct FRoadCrossChunkRecord {
    uint32_t localSegmentId;
    uint8_t isStart; // 1 if connection is at start, 0 if at end
    uint8_t neighborIsStart; // 1 if neighbor connection is at start, 0 if at end
    uint16_t pad;
    int32_t neighborChunkX;
    int32_t neighborChunkZ;
    uint32_t neighborSegmentId;
    float boundaryPos[3]; // Exact 3D seam coordinate
};

// Legacy Version 1 Chunk Header
struct FRoadChunkHeaderV1 {
    char magic[4]; // "FROA"
    uint32_t version; // 1
    float minX, maxX; // -250.0f, +250.0f
    float minZ, maxZ; // -250.0f, +250.0f

    uint32_t numPaletteEntries;
    uint32_t numSplinePoints;
    uint32_t numSegments;
    uint32_t numJunctions;

    uint32_t vertexCount;
    uint32_t indexCount;
};

// Legacy Version 2 Chunk Header (512m Cartesian Grid)
struct FRoadChunkHeaderV2 {
    char magic[4]; // "FROA"
    uint32_t version; // 2
    int32_t chunkX;   // Grid coordinate X
    int32_t chunkZ;   // Grid coordinate Z
    float minX, maxX; // Chunk world-space bounding box
    float minZ, maxZ; // Chunk world-space bounding box

    uint32_t numPaletteEntries;
    uint32_t numSplinePoints;
    uint32_t numSegments;
    uint32_t numJunctions;
    uint32_t numCrossChunkConnections;

    uint32_t vertexCount;
    uint32_t indexCount;
};

// Quadsphere Level 15 Chunk Header (Version 3 with 64-bit ECEF RTE anchor)
struct FRoadChunkHeader {
    char magic[4]; // "FROA"
    uint32_t version; // 3 (Quadsphere), 2 (Cartesian 512m), 1 (Legacy 500m)
    int32_t chunkX;   // Quadsphere tileX (or Cartesian chunkX)
    int32_t chunkZ;   // Quadsphere tileY (or Cartesian chunkZ)
    float minX, maxX; // Quadsphere minU, maxU (or Cartesian minX, maxX)
    float minZ, maxZ; // Quadsphere minV, maxV (or Cartesian minZ, maxZ)

    uint32_t numPaletteEntries;
    uint32_t numSplinePoints;
    uint32_t numSegments;
    uint32_t numJunctions;
    uint32_t numCrossChunkConnections; // Number of FRoadCrossChunkRecord entries

    // Pre-triangulated mesh counts (for v3: vertex positions are relative to chunkCenterECEF in meters)
    uint32_t vertexCount;
    uint32_t indexCount;

    // Quadsphere Level 15 metadata (Version 3)
    uint8_t  face;               // Cube face (0..5)
    uint8_t  lod;                // Quadtree level (default: 15)
    uint16_t reserved16;         // Alignment padding
    double   chunkCenterECEF[3]; // 64-bit ECEF anchor of chunk center O_chunk
};

#pragma pack(pop)

// Runtime In-Engine Structures
struct CrossChunkConnection {
    bool hasNeighbor = false;
    uint8_t neighborFace = 0;
    int neighborChunkX = 0; // tileX
    int neighborChunkZ = 0; // tileY
    uint32_t neighborSegmentId = 0;
    bool neighborIsStart = true;
    glm::vec3 boundaryPos{0.0f};      // Relative to chunk center
    glm::dvec3 boundaryPosECEF{0.0};  // 64-bit ECEF position
};
struct RoadSplinePoint {
    glm::vec3 pos{0.0f};        // Metric position relative to chunkCenterECEF
    glm::vec3 tangent{0.0f};    // Unit forward tangent vector
    glm::vec3 normal{0.0f};     // Outward sphere normal vector
    float distance = 0.0f;      // Accumulated arc length from start (meters)
    glm::dvec3 posECEF{0.0};    // Absolute 64-bit ECEF position
};

struct RawRoadFeature {
    uint32_t id = 0;
    std::string osmId;
    std::string name;
    std::string highway;
    std::string ref;
    std::string destination;
    std::string shoulderTag;
    float parsedWidth = -1.0f;
    float parsedLaneWidth = -1.0f;
    float parsedShoulderWidth = -1.0f;
    bool hasExplicitLanes = false;
    int lanes = 1;
    bool oneway = false;
    int layer = 0;
    bool isBridge = false;
    bool isTunnel = false;
    float elevation = 0.05f;
    std::vector<glm::vec3> rawPoints;   // Local metric coordinates
    std::vector<glm::dvec2> gpsPoints;  // Full 64-bit WGS84 (lon, lat)
};

struct RoadJunctionConnection {
    int segmentIdx = -1;
    bool isStart = true; // true if connection is at controlPoints[0], false if at controlPoints.back()
};

struct RoadJunction {
    uint32_t id = 0;
    glm::vec3 position{0.0f};
    std::vector<RoadJunctionConnection> connections;

    // Planar polygon plate geometry capping intersecting road ribbons
    std::vector<Vertex> patchVertices;
    std::vector<uint32_t> patchIndices;
    std::shared_ptr<Mesh> patchMesh;
};

struct ProcessedRoadSegment {
    uint32_t id = 0;
    std::string osmId;
    std::string name;
    std::string highway;
    std::string ref;
    int lanes = 1;
    bool oneway = false;
    int layer = 0;
    bool isBridge = false;
    bool isTunnel = false;
    float elevation = 0.05f;

    float laneWidth = 3.65f;
    float shoulderWidth = 1.5f;
    float totalWidth = 7.0f;
    float length = 0.0f;
    float sampleStep = 2.5f;
    std::string widthSource = "AASHTO Standard";

    // Topological Junction Connectivity
    int startJunctionId = -1;
    int endJunctionId = -1;
    float startSetback = 0.0f; // Distance (meters) to trim ribbon back from start junction
    float endSetback = 0.0f;   // Distance (meters) to trim ribbon back from end junction

    // Multi-chunk domain decomposition & cross-chunk topology
    uint8_t face = 2; // Quadsphere cube face
    int chunkX = 0;   // tileX (or legacy chunkX)
    int chunkZ = 0;   // tileY (or legacy chunkZ)
    glm::dvec3 chunkCenterECEF{0.0};
    CrossChunkConnection startNeighbor;
    CrossChunkConnection endNeighbor;

    // Editable raw waypoints / control points in local metric coordinates
    std::vector<glm::vec3> controlPoints;
    std::vector<glm::dvec2> gpsPoints; // 64-bit WGS84 (lon, lat)

    std::vector<RoadSplinePoint> splinePoints;
    std::shared_ptr<Mesh> visualMesh;

    bool in500mChunk = false;
};

#endif // ROAD_TYPES_H

