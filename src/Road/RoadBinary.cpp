#include "Road/RoadBinary.h"
#include "Road/RoadGenerator.h"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <unordered_set>

bool RoadBinary::saveQuadsphereChunk(const std::string& filepath,
                                    uint8_t face, uint16_t lod,
                                    uint32_t tileX, uint32_t tileY,
                                    const glm::dvec3& chunkCenterECEF,
                                    float minU, float maxU,
                                    float minV, float maxV,
                                    const std::vector<ProcessedRoadSegment>& segments,
                                    const std::vector<RoadJunction>& junctions) {
    FILE* file = std::fopen(filepath.c_str(), "wb");
    if (!file) {
        std::cerr << "RoadBinary: Failed to open file for writing: " << filepath << std::endl;
        return false;
    }

    FRoadChunkHeader header = {};
    header.magic[0] = 'F';
    header.magic[1] = 'R';
    header.magic[2] = 'O';
    header.magic[3] = 'A';
    header.version = 3;
    header.chunkX = static_cast<int32_t>(tileX);
    header.chunkZ = static_cast<int32_t>(tileY);
    header.minX = minU;
    header.maxX = maxU;
    header.minZ = minV;
    header.maxZ = maxV;
    header.face = face;
    header.lod = static_cast<uint8_t>(lod);
    header.reserved16 = 0;
    header.chunkCenterECEF[0] = chunkCenterECEF.x;
    header.chunkCenterECEF[1] = chunkCenterECEF.y;
    header.chunkCenterECEF[2] = chunkCenterECEF.z;

    // Filter valid segments
    std::vector<const ProcessedRoadSegment*> chunkSegments;
    for (const auto& seg : segments) {
        if (seg.splinePoints.size() >= 2) {
            chunkSegments.push_back(&seg);
        }
    }

    // Build Palette
    std::vector<LaneLayoutPaletteEntry> palette(4);
    palette[0].elementCount = 3;
    palette[0].elements[0] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 1.5f, -0.02f, 45 };
    palette[0].elements[1] = { LaneType::DrivableVehicle, MarkingType::None, 3.65f, -0.02f, 45 };
    palette[0].elements[2] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 1.5f, -0.02f, 45 };

    palette[1].elementCount = 4;
    palette[1].elements[0] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 2.0f, -0.02f, 55 };
    palette[1].elements[1] = { LaneType::DrivableVehicle, MarkingType::DashedWhite, 3.65f, -0.02f, 55 };
    palette[1].elements[2] = { LaneType::DrivableVehicle, MarkingType::None, 3.65f, -0.02f, 55 };
    palette[1].elements[3] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 2.0f, -0.02f, 55 };

    palette[2].elementCount = 4;
    palette[2].elements[0] = { LaneType::BarrierJersey, MarkingType::SolidWhite, 1.0f, 0.0f, 65 };
    palette[2].elements[1] = { LaneType::DrivableVehicle, MarkingType::DashedWhite, 3.65f, -0.02f, 65 };
    palette[2].elements[2] = { LaneType::DrivableVehicle, MarkingType::None, 3.65f, -0.02f, 65 };
    palette[2].elements[3] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 3.0f, -0.02f, 65 };

    palette[3].elementCount = 3;
    palette[3].elements[0] = { LaneType::Sidewalk, MarkingType::None, 1.5f, 0.0f, 25 };
    palette[3].elements[1] = { LaneType::DrivableVehicle, MarkingType::DoubleSolidYellow, 3.0f, -0.02f, 25 };
    palette[3].elements[2] = { LaneType::DrivableVehicle, MarkingType::None, 3.0f, -0.02f, 25 };

    header.numPaletteEntries = (uint32_t)palette.size();

    // Pool Spline Points & Segment Records
    std::vector<FRoadSamplePoint> splinePool;
    std::vector<FRoadSegmentRecord> segmentRecords;

    for (const auto* seg : chunkSegments) {
        FRoadSegmentRecord rec = {};
        rec.id = seg->id;
        rec.splinePointOffset = (uint32_t)splinePool.size();
        rec.splinePointCount = (uint16_t)seg->splinePoints.size();

        if (seg->highway == "trunk_link" || seg->highway == "primary_link") rec.layoutProfileId = 0;
        else if (seg->highway == "trunk" || seg->highway == "motorway") rec.layoutProfileId = 2;
        else if (seg->highway == "residential" || seg->highway == "unclassified" || seg->highway == "service") rec.layoutProfileId = 3;
        else rec.layoutProfileId = 1;

        rec.flags = seg->oneway ? 0x04 : 0x00;
        if (seg->isBridge || seg->layer > 0) rec.flags |= 0x01;
        if (seg->isTunnel || seg->layer < 0) rec.flags |= 0x02;
        rec.startJunctionId = (seg->startJunctionId >= 0) ? (uint32_t)seg->startJunctionId : 0xFFFFFFFF;
        rec.endJunctionId = (seg->endJunctionId >= 0) ? (uint32_t)seg->endJunctionId : 0xFFFFFFFF;
        segmentRecords.push_back(rec);

        for (const auto& sp : seg->splinePoints) {
            FRoadSamplePoint pt = {};
            pt.position[0] = sp.pos.x;
            pt.position[1] = sp.pos.y;
            pt.position[2] = sp.pos.z;

            pt.tangent[0] = sp.tangent.x;
            pt.tangent[1] = sp.tangent.y;
            pt.tangent[2] = sp.tangent.z;

            pt.normal[0] = sp.normal.x;
            pt.normal[1] = sp.normal.y;
            pt.normal[2] = sp.normal.z;

            pt.laneWidth = seg->laneWidth;
            pt.profileFlags = 0;
            splinePool.push_back(pt);
        }
    }

    // Build Junction Records only for junctions with connections inside this chunk
    std::vector<FRoadJunctionRecord> junctionRecords;
    for (const auto& junc : junctions) {
        FRoadJunctionRecord jr = {};
        jr.id = junc.id;
        jr.position[0] = junc.position.x;
        jr.position[1] = junc.position.y;
        jr.position[2] = junc.position.z;
        uint8_t count = 0;
        for (const auto* seg : chunkSegments) {
            if (seg->startJunctionId == (int)junc.id || seg->endJunctionId == (int)junc.id) {
                if (count < 8) {
                    jr.connectedSegmentIds[count++] = seg->id;
                }
            }
        }
        jr.connectionCount = count;
        if (jr.connectionCount > 0) {
            junctionRecords.push_back(jr);
        }
    }

    // Collect cross-chunk connection records
    std::vector<FRoadCrossChunkRecord> crossRecords;
    for (const auto* seg : chunkSegments) {
        if (seg->startNeighbor.hasNeighbor) {
            FRoadCrossChunkRecord r = {};
            r.localSegmentId = seg->id;
            r.isStart = 1;
            r.neighborIsStart = seg->startNeighbor.neighborIsStart ? 1 : 0;
            r.neighborChunkX = seg->startNeighbor.neighborChunkX;
            r.neighborChunkZ = seg->startNeighbor.neighborChunkZ;
            r.neighborSegmentId = seg->startNeighbor.neighborSegmentId;
            r.boundaryPos[0] = seg->startNeighbor.boundaryPos.x;
            r.boundaryPos[1] = seg->startNeighbor.boundaryPos.y;
            r.boundaryPos[2] = seg->startNeighbor.boundaryPos.z;
            crossRecords.push_back(r);
        }
        if (seg->endNeighbor.hasNeighbor) {
            FRoadCrossChunkRecord r = {};
            r.localSegmentId = seg->id;
            r.isStart = 0;
            r.neighborIsStart = seg->endNeighbor.neighborIsStart ? 1 : 0;
            r.neighborChunkX = seg->endNeighbor.neighborChunkX;
            r.neighborChunkZ = seg->endNeighbor.neighborChunkZ;
            r.neighborSegmentId = seg->endNeighbor.neighborSegmentId;
            r.boundaryPos[0] = seg->endNeighbor.boundaryPos.x;
            r.boundaryPos[1] = seg->endNeighbor.boundaryPos.y;
            r.boundaryPos[2] = seg->endNeighbor.boundaryPos.z;
            crossRecords.push_back(r);
        }
    }

    header.numCrossChunkConnections = (uint32_t)crossRecords.size();
    header.numSplinePoints = (uint32_t)splinePool.size();
    header.numSegments = (uint32_t)segmentRecords.size();
    header.numJunctions = (uint32_t)junctionRecords.size();

    // Generate pre-triangulated mesh for this chunk
    std::vector<Vertex> chunkVertices;
    std::vector<uint32_t> chunkIndices;
    for (const auto* seg : chunkSegments) {
        auto trimmed = RoadGenerator::trimSpline(seg->splinePoints, seg->startSetback, seg->endSetback);
        std::vector<Vertex> segVerts;
        std::vector<uint32_t> segIndices;
        RoadGenerator::generateRibbonGeometry(trimmed, seg->totalWidth,
                                              seg->lanes, seg->oneway, seg->shoulderWidth,
                                              segVerts, segIndices);
        RoadGenerator::clipRibbonAgainstJunctions(segVerts, *seg, junctions, segments);
        uint32_t baseVertex = (uint32_t)chunkVertices.size();
        for (const auto& v : segVerts) chunkVertices.push_back(v);
        for (auto idx : segIndices) chunkIndices.push_back(baseVertex + idx);
    }

    // Include junction patch plates inside this chunk
    for (const auto& junc : junctions) {
        if (junc.patchVertices.empty() || junc.patchIndices.empty()) continue;
        uint32_t baseVertex = (uint32_t)chunkVertices.size();
        for (const auto& v : junc.patchVertices) {
            chunkVertices.push_back(v);
        }
        for (auto idx : junc.patchIndices) {
            chunkIndices.push_back(baseVertex + idx);
        }
    }

    header.vertexCount = (uint32_t)chunkVertices.size();
    header.indexCount = (uint32_t)chunkIndices.size();

    // 1. Write Header
    std::fwrite(&header, sizeof(FRoadChunkHeader), 1, file);
    // 2. Write Palette
    std::fwrite(palette.data(), sizeof(LaneLayoutPaletteEntry), palette.size(), file);
    // 3. Write Spline Pool
    if (!splinePool.empty()) {
        std::fwrite(splinePool.data(), sizeof(FRoadSamplePoint), splinePool.size(), file);
    }
    // 4. Write Segment Records
    if (!segmentRecords.empty()) {
        std::fwrite(segmentRecords.data(), sizeof(FRoadSegmentRecord), segmentRecords.size(), file);
    }
    // 5. Write Junction Records
    if (!junctionRecords.empty()) {
        std::fwrite(junctionRecords.data(), sizeof(FRoadJunctionRecord), junctionRecords.size(), file);
    }
    // 6. Write Cross-Chunk Connection Records
    if (!crossRecords.empty()) {
        std::fwrite(crossRecords.data(), sizeof(FRoadCrossChunkRecord), crossRecords.size(), file);
    }
    // 7. Write Pre-triangulated Mesh
    if (!chunkVertices.empty()) {
        std::fwrite(chunkVertices.data(), sizeof(Vertex), chunkVertices.size(), file);
    }
    if (!chunkIndices.empty()) {
        std::fwrite(chunkIndices.data(), sizeof(uint32_t), chunkIndices.size(), file);
    }

    std::fclose(file);
    return true;
}

bool RoadBinary::saveChunk(const std::string& filepath,
                          int chunkX, int chunkZ,
                          float minX, float maxX,
                          float minZ, float maxZ,
                          const std::vector<ProcessedRoadSegment>& segments,
                          const std::vector<RoadJunction>& junctions) {
    FILE* file = std::fopen(filepath.c_str(), "wb");
    if (!file) {
        std::cerr << "RoadBinary: Failed to open file for writing: " << filepath << std::endl;
        return false;
    }

    FRoadChunkHeaderV2 header = {};
    header.magic[0] = 'F';
    header.magic[1] = 'R';
    header.magic[2] = 'O';
    header.magic[3] = 'A';
    header.version = 2;
    header.chunkX = chunkX;
    header.chunkZ = chunkZ;
    header.minX = minX;
    header.maxX = maxX;
    header.minZ = minZ;
    header.maxZ = maxZ;

    // Filter valid segments
    std::vector<const ProcessedRoadSegment*> chunkSegments;
    for (const auto& seg : segments) {
        if (seg.splinePoints.size() >= 2) {
            chunkSegments.push_back(&seg);
        }
    }

    // Build Palette
    std::vector<LaneLayoutPaletteEntry> palette(4);
    // Profile 0: 1-lane link
    palette[0].elementCount = 3;
    palette[0].elements[0] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 1.5f, -0.02f, 45 };
    palette[0].elements[1] = { LaneType::DrivableVehicle, MarkingType::None, 3.65f, -0.02f, 45 };
    palette[0].elements[2] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 1.5f, -0.02f, 45 };

    // Profile 1: 2-lane trunk
    palette[1].elementCount = 4;
    palette[1].elements[0] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 2.0f, -0.02f, 55 };
    palette[1].elements[1] = { LaneType::DrivableVehicle, MarkingType::DashedWhite, 3.65f, -0.02f, 55 };
    palette[1].elements[2] = { LaneType::DrivableVehicle, MarkingType::None, 3.65f, -0.02f, 55 };
    palette[1].elements[3] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 2.0f, -0.02f, 55 };

    // Profile 2: 4-lane divided
    palette[2].elementCount = 4;
    palette[2].elements[0] = { LaneType::BarrierJersey, MarkingType::SolidWhite, 1.0f, 0.0f, 65 };
    palette[2].elements[1] = { LaneType::DrivableVehicle, MarkingType::DashedWhite, 3.65f, -0.02f, 65 };
    palette[2].elements[2] = { LaneType::DrivableVehicle, MarkingType::None, 3.65f, -0.02f, 65 };
    palette[2].elements[3] = { LaneType::ShoulderPaved, MarkingType::SolidWhite, 3.0f, -0.02f, 65 };

    // Profile 3: 2-lane residential
    palette[3].elementCount = 3;
    palette[3].elements[0] = { LaneType::Sidewalk, MarkingType::None, 1.5f, 0.0f, 25 };
    palette[3].elements[1] = { LaneType::DrivableVehicle, MarkingType::DoubleSolidYellow, 3.0f, -0.02f, 25 };
    palette[3].elements[2] = { LaneType::DrivableVehicle, MarkingType::None, 3.0f, -0.02f, 25 };

    header.numPaletteEntries = (uint32_t)palette.size();

    // Pool Spline Points & Segment Records
    std::vector<FRoadSamplePoint> splinePool;
    std::vector<FRoadSegmentRecord> segmentRecords;

    for (const auto* seg : chunkSegments) {
        FRoadSegmentRecord rec = {};
        rec.id = seg->id;
        rec.splinePointOffset = (uint32_t)splinePool.size();
        rec.splinePointCount = (uint16_t)seg->splinePoints.size();

        if (seg->highway == "trunk_link" || seg->highway == "primary_link") rec.layoutProfileId = 0;
        else if (seg->highway == "trunk" || seg->highway == "motorway") rec.layoutProfileId = 2;
        else if (seg->highway == "residential" || seg->highway == "unclassified" || seg->highway == "service") rec.layoutProfileId = 3;
        else rec.layoutProfileId = 1;

        rec.flags = seg->oneway ? 0x04 : 0x00;
        if (seg->isBridge || seg->layer > 0) rec.flags |= 0x01;
        if (seg->isTunnel || seg->layer < 0) rec.flags |= 0x02;
        rec.startJunctionId = (seg->startJunctionId >= 0) ? (uint32_t)seg->startJunctionId : 0xFFFFFFFF;
        rec.endJunctionId = (seg->endJunctionId >= 0) ? (uint32_t)seg->endJunctionId : 0xFFFFFFFF;
        segmentRecords.push_back(rec);

        for (const auto& sp : seg->splinePoints) {
            FRoadSamplePoint pt = {};
            pt.position[0] = sp.pos.x;
            pt.position[1] = sp.pos.y;
            pt.position[2] = sp.pos.z;

            pt.tangent[0] = sp.tangent.x;
            pt.tangent[1] = sp.tangent.y;
            pt.tangent[2] = sp.tangent.z;

            pt.normal[0] = sp.normal.x;
            pt.normal[1] = sp.normal.y;
            pt.normal[2] = sp.normal.z;

            pt.laneWidth = seg->laneWidth;
            pt.profileFlags = 0;
            splinePool.push_back(pt);
        }
    }

    // Collect segment IDs belonging to this chunk
    std::unordered_set<uint32_t> chunkSegIds;
    for (const auto* seg : chunkSegments) {
        chunkSegIds.insert(seg->id);
    }

    // Build Junction Records only for junctions with connections inside this chunk
    std::vector<FRoadJunctionRecord> junctionRecords;
    for (const auto& junc : junctions) {
        FRoadJunctionRecord jr = {};
        jr.id = junc.id;
        jr.position[0] = junc.position.x;
        jr.position[1] = junc.position.y;
        jr.position[2] = junc.position.z;
        uint8_t count = 0;
        for (const auto* seg : chunkSegments) {
            if (seg->startJunctionId == (int)junc.id || seg->endJunctionId == (int)junc.id) {
                if (count < 8) {
                    jr.connectedSegmentIds[count++] = seg->id;
                }
            }
        }
        jr.connectionCount = count;
        if (jr.connectionCount > 0) {
            junctionRecords.push_back(jr);
        }
    }

    // Collect cross-chunk connection records
    std::vector<FRoadCrossChunkRecord> crossRecords;
    for (const auto* seg : chunkSegments) {
        if (seg->startNeighbor.hasNeighbor) {
            FRoadCrossChunkRecord r = {};
            r.localSegmentId = seg->id;
            r.isStart = 1;
            r.neighborIsStart = seg->startNeighbor.neighborIsStart ? 1 : 0;
            r.neighborChunkX = seg->startNeighbor.neighborChunkX;
            r.neighborChunkZ = seg->startNeighbor.neighborChunkZ;
            r.neighborSegmentId = seg->startNeighbor.neighborSegmentId;
            r.boundaryPos[0] = seg->startNeighbor.boundaryPos.x;
            r.boundaryPos[1] = seg->startNeighbor.boundaryPos.y;
            r.boundaryPos[2] = seg->startNeighbor.boundaryPos.z;
            crossRecords.push_back(r);
        }
        if (seg->endNeighbor.hasNeighbor) {
            FRoadCrossChunkRecord r = {};
            r.localSegmentId = seg->id;
            r.isStart = 0;
            r.neighborIsStart = seg->endNeighbor.neighborIsStart ? 1 : 0;
            r.neighborChunkX = seg->endNeighbor.neighborChunkX;
            r.neighborChunkZ = seg->endNeighbor.neighborChunkZ;
            r.neighborSegmentId = seg->endNeighbor.neighborSegmentId;
            r.boundaryPos[0] = seg->endNeighbor.boundaryPos.x;
            r.boundaryPos[1] = seg->endNeighbor.boundaryPos.y;
            r.boundaryPos[2] = seg->endNeighbor.boundaryPos.z;
            crossRecords.push_back(r);
        }
    }
    header.numCrossChunkConnections = (uint32_t)crossRecords.size();

    header.numSplinePoints = (uint32_t)splinePool.size();
    header.numSegments = (uint32_t)segmentRecords.size();
    header.numJunctions = (uint32_t)junctionRecords.size();

    // Generate pre-triangulated mesh for this chunk
    std::vector<Vertex> chunkVertices;
    std::vector<uint32_t> chunkIndices;
    for (const auto* seg : chunkSegments) {
        auto trimmed = RoadGenerator::trimSpline(seg->splinePoints, seg->startSetback, seg->endSetback);
        std::vector<Vertex> segVerts;
        std::vector<uint32_t> segIndices;
        RoadGenerator::generateRibbonGeometry(trimmed, seg->totalWidth,
                                              seg->lanes, seg->oneway, seg->shoulderWidth,
                                              segVerts, segIndices);
        RoadGenerator::clipRibbonAgainstJunctions(segVerts, *seg, junctions, segments);
        uint32_t baseVertex = (uint32_t)chunkVertices.size();
        for (const auto& v : segVerts) chunkVertices.push_back(v);
        for (auto idx : segIndices) chunkIndices.push_back(baseVertex + idx);
    }

    // Include junction patch plates inside this chunk
    for (const auto& junc : junctions) {
        if (junc.patchVertices.empty() || junc.patchIndices.empty()) continue;
        if (junc.position.x >= minX - 1.0f && junc.position.x <= maxX + 1.0f &&
            junc.position.z >= minZ - 1.0f && junc.position.z <= maxZ + 1.0f) {
            uint32_t baseVertex = (uint32_t)chunkVertices.size();
            for (const auto& v : junc.patchVertices) {
                chunkVertices.push_back(v);
            }
            for (auto idx : junc.patchIndices) {
                chunkIndices.push_back(baseVertex + idx);
            }
        }
    }

    header.vertexCount = (uint32_t)chunkVertices.size();
    header.indexCount = (uint32_t)chunkIndices.size();

    // 1. Write Header
    std::fwrite(&header, sizeof(FRoadChunkHeaderV2), 1, file);

    // 2. Write Palette
    std::fwrite(palette.data(), sizeof(LaneLayoutPaletteEntry), palette.size(), file);

    // 3. Write Spline Pool
    if (!splinePool.empty()) {
        std::fwrite(splinePool.data(), sizeof(FRoadSamplePoint), splinePool.size(), file);
    }

    // 4. Write Segment Records
    if (!segmentRecords.empty()) {
        std::fwrite(segmentRecords.data(), sizeof(FRoadSegmentRecord), segmentRecords.size(), file);
    }

    // 5. Write Junction Records
    if (!junctionRecords.empty()) {
        std::fwrite(junctionRecords.data(), sizeof(FRoadJunctionRecord), junctionRecords.size(), file);
    }

    // 6. Write Cross-Chunk Connection Records
    if (!crossRecords.empty()) {
        std::fwrite(crossRecords.data(), sizeof(FRoadCrossChunkRecord), crossRecords.size(), file);
    }

    // 7. Write Pre-triangulated Mesh
    if (!chunkVertices.empty()) {
        std::fwrite(chunkVertices.data(), sizeof(Vertex), chunkVertices.size(), file);
    }
    if (!chunkIndices.empty()) {
        std::fwrite(chunkIndices.data(), sizeof(uint32_t), chunkIndices.size(), file);
    }

    std::fclose(file);
    return true;
}

bool RoadBinary::save500mChunk(const std::string& filepath,
                              const std::vector<ProcessedRoadSegment>& segments,
                              const std::vector<RoadJunction>& junctions) {
    std::vector<ProcessedRoadSegment> filtered;
    for (const auto& s : segments) {
        if (s.in500mChunk) filtered.push_back(s);
    }
    if (filtered.empty()) filtered = segments;
    return saveChunk(filepath, 0, 0, -256.0f, 256.0f, -256.0f, 256.0f, filtered, junctions);
}

bool RoadBinary::loadChunk(const std::string& filepath,
                          std::vector<ProcessedRoadSegment>& outSegments,
                          std::shared_ptr<Mesh>& outMergedMesh,
                          FRoadChunkHeader& outHeader,
                          std::vector<RoadJunction>* outJunctions) {
    outSegments.clear();
    outMergedMesh = nullptr;
    if (outJunctions) outJunctions->clear();

    FILE* file = std::fopen(filepath.c_str(), "rb");
    if (!file) {
        return false;
    }

    char magic[4];
    uint32_t version = 0;
    if (std::fread(magic, 4, 1, file) != 1 || std::fread(&version, sizeof(uint32_t), 1, file) != 1) {
        std::fclose(file);
        return false;
    }

    if (magic[0] != 'F' || magic[1] != 'R' || magic[2] != 'O' || magic[3] != 'A') {
        std::cerr << "RoadBinary: Invalid magic in " << filepath << std::endl;
        std::fclose(file);
        return false;
    }

    std::fseek(file, 0, SEEK_SET);
    std::vector<FRoadCrossChunkRecord> crossRecords;

    if (version == 1) {
        FRoadChunkHeaderV1 h1 = {};
        if (std::fread(&h1, sizeof(FRoadChunkHeaderV1), 1, file) != 1) {
            std::fclose(file);
            return false;
        }
        std::memcpy(outHeader.magic, h1.magic, 4);
        outHeader.version = 1;
        outHeader.chunkX = 0;
        outHeader.chunkZ = 0;
        outHeader.minX = h1.minX;
        outHeader.maxX = h1.maxX;
        outHeader.minZ = h1.minZ;
        outHeader.maxZ = h1.maxZ;
        outHeader.numPaletteEntries = h1.numPaletteEntries;
        outHeader.numSplinePoints = h1.numSplinePoints;
        outHeader.numSegments = h1.numSegments;
        outHeader.numJunctions = h1.numJunctions;
        outHeader.numCrossChunkConnections = 0;
        outHeader.vertexCount = h1.vertexCount;
        outHeader.indexCount = h1.indexCount;
        outHeader.face = 0;
        outHeader.lod = 0;
        outHeader.reserved16 = 0;
        outHeader.chunkCenterECEF[0] = 0.0;
        outHeader.chunkCenterECEF[1] = 0.0;
        outHeader.chunkCenterECEF[2] = 0.0;
    } else if (version == 2) {
        FRoadChunkHeaderV2 h2 = {};
        if (std::fread(&h2, sizeof(FRoadChunkHeaderV2), 1, file) != 1) {
            std::fclose(file);
            return false;
        }
        std::memcpy(outHeader.magic, h2.magic, 4);
        outHeader.version = 2;
        outHeader.chunkX = h2.chunkX;
        outHeader.chunkZ = h2.chunkZ;
        outHeader.minX = h2.minX;
        outHeader.maxX = h2.maxX;
        outHeader.minZ = h2.minZ;
        outHeader.maxZ = h2.maxZ;
        outHeader.numPaletteEntries = h2.numPaletteEntries;
        outHeader.numSplinePoints = h2.numSplinePoints;
        outHeader.numSegments = h2.numSegments;
        outHeader.numJunctions = h2.numJunctions;
        outHeader.numCrossChunkConnections = h2.numCrossChunkConnections;
        outHeader.vertexCount = h2.vertexCount;
        outHeader.indexCount = h2.indexCount;
        outHeader.face = 0;
        outHeader.lod = 0;
        outHeader.reserved16 = 0;
        outHeader.chunkCenterECEF[0] = 0.0;
        outHeader.chunkCenterECEF[1] = 0.0;
        outHeader.chunkCenterECEF[2] = 0.0;
    } else {
        if (std::fread(&outHeader, sizeof(FRoadChunkHeader), 1, file) != 1) {
            std::fclose(file);
            return false;
        }
    }

    // Read Palette
    std::vector<LaneLayoutPaletteEntry> palette(outHeader.numPaletteEntries);
    if (outHeader.numPaletteEntries > 0) {
        std::fread(palette.data(), sizeof(LaneLayoutPaletteEntry), outHeader.numPaletteEntries, file);
    }

    // Read Spline Pool
    std::vector<FRoadSamplePoint> splinePool(outHeader.numSplinePoints);
    if (outHeader.numSplinePoints > 0) {
        std::fread(splinePool.data(), sizeof(FRoadSamplePoint), outHeader.numSplinePoints, file);
    }

    // Read Segment Records
    std::vector<FRoadSegmentRecord> records(outHeader.numSegments);
    if (outHeader.numSegments > 0) {
        std::fread(records.data(), sizeof(FRoadSegmentRecord), outHeader.numSegments, file);
    }

    // Read Junction Records
    std::vector<FRoadJunctionRecord> junctionRecords(outHeader.numJunctions);
    if (outHeader.numJunctions > 0) {
        std::fread(junctionRecords.data(), sizeof(FRoadJunctionRecord), outHeader.numJunctions, file);
    }

    // Read Cross-Chunk Records (if version >= 2)
    if (outHeader.version >= 2 && outHeader.numCrossChunkConnections > 0) {
        crossRecords.resize(outHeader.numCrossChunkConnections);
        std::fread(crossRecords.data(), sizeof(FRoadCrossChunkRecord), outHeader.numCrossChunkConnections, file);
    }

    // Read Pre-triangulated Mesh
    std::vector<Vertex> vertices(outHeader.vertexCount);
    std::vector<uint32_t> indices(outHeader.indexCount);

    if (outHeader.vertexCount > 0) {
        std::fread(vertices.data(), sizeof(Vertex), outHeader.vertexCount, file);
    }
    if (outHeader.indexCount > 0) {
        std::fread(indices.data(), sizeof(uint32_t), outHeader.indexCount, file);
    }
    std::fclose(file);

    if (!vertices.empty() && !indices.empty()) {
        outMergedMesh = std::make_shared<Mesh>(vertices.data(), (Uint32)vertices.size(),
                                               indices.data(), (Uint32)indices.size());
    }

    // Reconstruct ProcessedRoadSegments
    for (const auto& rec : records) {
        ProcessedRoadSegment seg;
        seg.id = rec.id;
        seg.osmId = "compiled/" + std::to_string(rec.id);
        seg.face = outHeader.face;
        seg.chunkX = outHeader.chunkX;
        seg.chunkZ = outHeader.chunkZ;
        seg.chunkCenterECEF = glm::dvec3(outHeader.chunkCenterECEF[0], outHeader.chunkCenterECEF[1], outHeader.chunkCenterECEF[2]);
        seg.oneway = (rec.flags & 0x04) != 0;
        seg.isBridge = (rec.flags & 0x01) != 0;
        seg.isTunnel = (rec.flags & 0x02) != 0;
        seg.layer = seg.isBridge ? 1 : (seg.isTunnel ? -1 : 0);
        seg.elevation = seg.isBridge ? 5.5f : (seg.isTunnel ? -5.0f : 0.05f);
        seg.in500mChunk = true;
        seg.startJunctionId = (rec.startJunctionId != 0xFFFFFFFF) ? (int)rec.startJunctionId : -1;
        seg.endJunctionId = (rec.endJunctionId != 0xFFFFFFFF) ? (int)rec.endJunctionId : -1;

        if (rec.layoutProfileId < palette.size()) {
            const auto& pal = palette[rec.layoutProfileId];
            float totalW = 0.0f;
            int laneCount = 0;
            for (uint8_t e = 0; e < pal.elementCount; ++e) {
                totalW += pal.elements[e].width;
                if (pal.elements[e].type == LaneType::DrivableVehicle) laneCount++;
            }
            seg.lanes = std::max(1, laneCount);
            seg.totalWidth = totalW;
            seg.laneWidth = 3.65f;
            seg.shoulderWidth = 1.5f;
        }

        uint32_t start = rec.splinePointOffset;
        uint32_t count = rec.splinePointCount;
        float accDist = 0.0f;
        for (uint32_t p = 0; p < count && (start + p) < splinePool.size(); ++p) {
            const auto& sp = splinePool[start + p];
            glm::vec3 pos(sp.position[0], sp.position[1], sp.position[2]);
            glm::vec3 tan(sp.tangent[0], sp.tangent[1], sp.tangent[2]);
            glm::vec3 norm(sp.normal[0], sp.normal[1], sp.normal[2]);

            if (p > 0) {
                const auto& prev = splinePool[start + p - 1];
                accDist += glm::length(pos - glm::vec3(prev.position[0], prev.position[1], prev.position[2]));
            }
            glm::dvec3 pECEF = (outHeader.version >= 3) ? (seg.chunkCenterECEF + glm::dvec3(pos)) : glm::dvec3(pos);
            seg.splinePoints.push_back({ pos, tan, norm, accDist, pECEF });
        }
        seg.length = accDist;

        // Populate control points for editor manipulation (endpoints + intermediate samples)
        if (!seg.splinePoints.empty()) {
            seg.controlPoints.push_back(seg.splinePoints.front().pos);
            size_t step = std::max((size_t)1, seg.splinePoints.size() / 5);
            for (size_t p = step; p + step < seg.splinePoints.size(); p += step) {
                seg.controlPoints.push_back(seg.splinePoints[p].pos);
            }
            seg.controlPoints.push_back(seg.splinePoints.back().pos);
        }

        outSegments.push_back(seg);
    }

    // Populate cross-chunk connections
    for (const auto& cr : crossRecords) {
        for (auto& s : outSegments) {
            if (s.id == cr.localSegmentId) {
                CrossChunkConnection conn;
                conn.hasNeighbor = true;
                conn.neighborChunkX = cr.neighborChunkX;
                conn.neighborChunkZ = cr.neighborChunkZ;
                conn.neighborSegmentId = cr.neighborSegmentId;
                conn.neighborIsStart = (cr.neighborIsStart != 0);
                conn.boundaryPos = glm::vec3(cr.boundaryPos[0], cr.boundaryPos[1], cr.boundaryPos[2]);
                if (cr.isStart) s.startNeighbor = conn;
                else s.endNeighbor = conn;
                break;
            }
        }
    }

    if (outJunctions) {
        for (const auto& jr : junctionRecords) {
            RoadJunction junc;
            junc.id = jr.id;
            junc.position = glm::vec3(jr.position[0], jr.position[1], jr.position[2]);
            for (uint8_t c = 0; c < jr.connectionCount; ++c) {
                uint32_t targetSegId = jr.connectedSegmentIds[c];
                for (size_t s = 0; s < outSegments.size(); ++s) {
                    if (outSegments[s].id == targetSegId) {
                        bool isStart = (outSegments[s].startJunctionId == (int)jr.id);
                        junc.connections.push_back({ (int)s, isStart });
                        break;
                    }
                }
            }
            outJunctions->push_back(junc);
        }
    }

    return true;
}
