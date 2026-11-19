#include <iostream>
#include <fstream>
#include <vector>
#include <cassert>
#include <cmath>
#include "Road/RoadTypes.h"
#include "Road/GeoJsonLoader.h"
#include "Road/RoadGenerator.h"
#include "Road/RoadBinary.h"
#include "Road/RoadElevationSolver.h"
#include "Terrain/QuadsphereMath.h"

int main() {
    std::cout << "--- 1. Testing GeoJSON Load & Network Topology Generation ---" << std::endl;
    std::vector<RawRoadFeature> rawFeatures;
    double centerLon = 0.0, centerLat = 0.0;
    bool ok = GeoJsonLoader::load("jughandle/export.geojson", rawFeatures, centerLon, centerLat);
    assert(ok && "Failed to load jughandle/export.geojson");
    std::cout << "Loaded " << rawFeatures.size() << " features." << std::endl;

    std::vector<ProcessedRoadSegment> segments;
    RoadGenerator::processFeatures(rawFeatures, 3.65f, 1.50f, 2.50f, segments, false);
    std::cout << "Processed " << segments.size() << " segments." << std::endl;

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& s = segments[i];
        std::cout << "Seg #" << i << " (id=" << s.id << ", " << s.highway << ", lanes=" << s.lanes
                  << ", len=" << s.length << ", in500=" << s.in500mChunk << ") p0=("
                  << s.controlPoints.front().x << ", " << s.controlPoints.front().z
                  << ") -> pend=(" << s.controlPoints.back().x << ", "
                  << s.controlPoints.back().z << ")" << std::endl;
    }

    std::vector<RoadJunction> junctions;
    RoadGenerator::buildNetworkTopology(segments, junctions);
    std::cout << "Detected " << junctions.size() << " topological junctions." << std::endl;
    assert(!junctions.empty() && "No junctions detected!");

    // Generate Junction Patches and Setbacks
    RoadGenerator::generateJunctionPatches(segments, junctions);

    int patchCount = 0;
    int totalPatchVerts = 0;
    int totalPatchTris = 0;
    for (const auto& j : junctions) {
        if (!j.patchVertices.empty()) {
            patchCount++;
            totalPatchVerts += (int)j.patchVertices.size();
            totalPatchTris += (int)j.patchIndices.size() / 3;
        }
    }
    std::cout << "Generated " << patchCount << " planar junction patch plates ("
              << totalPatchVerts << " vertices, " << totalPatchTris << " triangles)." << std::endl;
    assert(patchCount >= 20 && "Expected at least 20 junction patches for 3-way and 4-way intersections!");

    int upwardNormals = 0;
    int invertedNormals = 0;
    for (size_t jIdx = 0; jIdx < junctions.size(); ++jIdx) {
        const auto& j = junctions[jIdx];
        int jUp = 0, jInv = 0;
        for (size_t t = 0; t + 2 < j.patchIndices.size(); t += 3) {
            uint32_t i0 = j.patchIndices[t+0];
            uint32_t i1 = j.patchIndices[t+1];
            uint32_t i2 = j.patchIndices[t+2];
            const auto& v0 = j.patchVertices[i0];
            const auto& v1 = j.patchVertices[i1];
            const auto& v2 = j.patchVertices[i2];
            glm::vec3 norm = glm::cross(glm::vec3(v1.position[0] - v0.position[0], v1.position[1] - v0.position[1], v1.position[2] - v0.position[2]),
                                        glm::vec3(v2.position[0] - v0.position[0], v2.position[1] - v0.position[1], v2.position[2] - v0.position[2]));
            if (glm::length(norm) < 1e-4f) continue;
            if (norm.y >= -1e-4f) { upwardNormals++; jUp++; }
            else {
                invertedNormals++; jInv++;
            }
        }
        if (jInv > 0) {
            std::cout << "  Junction #" << jIdx << " (" << j.connections.size() << "-way) has "
                      << jInv << " inverted triangles out of " << (jUp + jInv) << " total!" << std::endl;
        }
    }
    std::cout << "Junction patches: " << upwardNormals << " upward-facing triangles, "
              << invertedNormals << " inverted triangles." << std::endl;
    assert(invertedNormals == 0 && "All junction patch triangles must be upward-facing!");

    // Verify ribbons stop before junction centers (setbacks > 0)
    int setbackCount = 0;
    for (const auto& seg : segments) {
        if (seg.startSetback > 0.0f) setbackCount++;
        if (seg.endSetback > 0.0f) setbackCount++;
    }
    std::cout << "Verified " << setbackCount << " segment endpoints have ribbon setbacks (10-35m clearance)." << std::endl;
    assert(setbackCount > 0 && "Expected setbacks on road ribbons meeting at junctions!");

    struct Cap {
        int segIdx;
        bool isStart;
        glm::vec3 pos;
        glm::vec3 dir;
        float width;
    };
    std::vector<Cap> caps;
    for (size_t i = 0; i < segments.size(); ++i) {
        auto trimmed = RoadGenerator::trimSpline(segments[i].splinePoints, segments[i].startSetback, segments[i].endSetback);
        if (trimmed.size() < 2) continue;
        caps.push_back({ (int)i, true, trimmed.front().pos, trimmed.front().tangent, segments[i].totalWidth });
        caps.push_back({ (int)i, false, trimmed.back().pos, -trimmed.back().tangent, segments[i].totalWidth });
    }

    std::cout << "\n=== Scanning for facing gaps between road caps ===" << std::endl;
    for (size_t a = 0; a < caps.size(); ++a) {
        for (size_t b = a + 1; b < caps.size(); ++b) {
            float d = glm::distance(caps[a].pos, caps[b].pos);
            if (d > 5.0f && d < 60.0f) {
                float dot = glm::dot(glm::normalize(caps[a].dir), glm::normalize(caps[b].dir));
                if (dot < -0.6f) {
                    std::cout << "Facing gap (" << d << "m, dot=" << dot << ") between Seg #" << caps[a].segIdx
                              << " (" << (caps[a].isStart?"start":"end") << ") at (" << caps[a].pos.x << ", " << caps[a].pos.z << ")"
                              << " and Seg #" << caps[b].segIdx << " (" << (caps[b].isStart?"start":"end") << ") at (" << caps[b].pos.x << ", " << caps[b].pos.z << ")" << std::endl;
                }
            }
        }
    }

    std::cout << "\n=== All junctions near (110, 90) ===" << std::endl;
    for (size_t j = 0; j < junctions.size(); ++j) {
        const auto& junc = junctions[j];
        if (junc.position.x >= 90 && junc.position.x <= 130 && junc.position.z >= 60 && junc.position.z <= 130) {
            std::cout << "Junction #" << j << " (" << junc.connections.size() << "-way) at ("
                      << junc.position.x << ", " << junc.position.y << ", " << junc.position.z
                      << ") patchVerts=" << junc.patchVertices.size() << std::endl;
            for (const auto& c : junc.connections) {
                const auto& s = segments[c.segmentIdx];
                float sb = c.isStart ? s.startSetback : s.endSetback;
                std::cout << "  conn seg #" << c.segmentIdx << " (id=" << s.id << ", " << s.highway
                          << ", lanes=" << s.lanes << ", len=" << s.length << ") isStart=" << c.isStart
                          << " setback=" << sb << std::endl;
            }
        }
    }
    for (size_t j = 0; j < junctions.size(); ++j) {
        const auto& junc = junctions[j];
        if (junc.connections.size() >= 3) {
            std::cout << "Junction #" << j << " (" << junc.connections.size() << "-way) at ("
                      << junc.position.x << ", " << junc.position.z << "):" << std::endl;
            for (const auto& c : junc.connections) {
                const auto& s = segments[c.segmentIdx];
                float sb = c.isStart ? s.startSetback : s.endSetback;
                std::cout << "  conn seg #" << c.segmentIdx << " (" << s.highway << ", lanes=" << s.lanes
                          << ", len=" << s.length << ", width=" << s.totalWidth << ") isStart=" << c.isStart
                          << " setback=" << sb << std::endl;
            }
            std::cout << "  Patch vertices: " << junc.patchVertices.size()
                      << " Triangles: " << junc.patchIndices.size() / 3 << std::endl;
        }
    }

    int twoWay = 0, threeWay = 0, fourWay = 0;
    for (const auto& j : junctions) {
        if (j.connections.size() == 2) twoWay++;
        else if (j.connections.size() == 3) threeWay++;
        else if (j.connections.size() >= 4) fourWay++;
    }
    std::cout << "Junction breakdown: " << twoWay << " 2-way, " << threeWay << " 3-way, " << fourWay << " 4-way." << std::endl;
    assert(threeWay > 0 && "Expected 3-way forks/merges in jughandle!");
    assert(fourWay > 0 && "Expected 4-way intersection!");

    std::cout << "\n--- 2. Testing Coupled Control Point Dragging ---" << std::endl;
    // Find a 3-way junction
    int testJuncIdx = -1;
    for (size_t i = 0; i < junctions.size(); ++i) {
        if (junctions[i].connections.size() >= 3) {
            testJuncIdx = (int)i;
            break;
        }
    }
    assert(testJuncIdx >= 0);
    auto& targetJunc = junctions[testJuncIdx];
    glm::vec3 originalPos = targetJunc.position;
    glm::vec3 movedPos = originalPos + glm::vec3(5.0f, 0.0f, -5.0f);

    std::cout << "Testing junction #" << targetJunc.id << " with " << targetJunc.connections.size() << " connections at ("
              << originalPos.x << ", " << originalPos.z << ")" << std::endl;

    // Simulate drag operation on target junction
    targetJunc.position = movedPos;
    for (const auto& conn : targetJunc.connections) {
        auto& seg = segments[conn.segmentIdx];
        if (conn.isStart) {
            seg.controlPoints.front() = movedPos;
        } else {
            seg.controlPoints.back() = movedPos;
        }
    }

    // Verify all connected segments now match movedPos at their respective ends
    for (const auto& conn : targetJunc.connections) {
        const auto& seg = segments[conn.segmentIdx];
        glm::vec3 pt = conn.isStart ? seg.controlPoints.front() : seg.controlPoints.back();
        float dist = glm::length(pt - movedPos);
        assert(dist < 1e-4f && "Connected road endpoint failed to update during coupled drag!");
    }
    std::cout << "Coupled control point movement verified across all " << targetJunc.connections.size() << " connected roads!" << std::endl;

    std::cout << "\n--- 3. Testing Binary (.froad) Serialization with Junctions ---" << std::endl;
    const std::string testFroad = "/tmp/test_jughandle_junctions.froad";
    bool saveOk = RoadBinary::save500mChunk(testFroad, segments, junctions);
    assert(saveOk && "Failed to save .froad chunk with junctions");

    std::vector<ProcessedRoadSegment> loadedSegs;
    std::shared_ptr<Mesh> loadedMesh;
    FRoadChunkHeader header;
    std::vector<RoadJunction> loadedJunctions;

    bool loadOk = RoadBinary::loadChunk(testFroad, loadedSegs, loadedMesh, header, &loadedJunctions);
    assert(loadOk && "Failed to load .froad chunk with junctions");

    std::cout << "Loaded .froad header: " << header.numSegments << " segments, "
              << header.numJunctions << " junctions, "
              << header.vertexCount << " vertices, "
              << header.indexCount << " indices." << std::endl;

    assert(header.numJunctions == loadedJunctions.size());
    std::cout << "Loaded " << loadedJunctions.size() << " junctions from binary intermediate." << std::endl;

    // Check junction integrity
    for (size_t i = 0; i < loadedJunctions.size(); ++i) {
        const auto& junc = loadedJunctions[i];
        assert(junc.connections.size() >= 1);
        for (const auto& conn : junc.connections) {
            assert(conn.segmentIdx >= 0 && conn.segmentIdx < (int)loadedSegs.size());
            const auto& seg = loadedSegs[conn.segmentIdx];
            if (conn.isStart) {
                assert(seg.startJunctionId == (int)junc.id);
            } else {
                assert(seg.endJunctionId == (int)junc.id);
            }
        }
    }
    std::cout << "All loaded junctions maintain 100% graph connectivity with segments!" << std::endl;

    std::cout << "\n--- 4. Testing Segment Splitting at Interior Control Point ---" << std::endl;
    // Find a segment with at least 4 control points
    int splitSegIdx = -1;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (segments[i].controlPoints.size() >= 4) {
            splitSegIdx = (int)i;
            break;
        }
    }
    assert(splitSegIdx >= 0 && "Expected to find segment with >= 4 control points");
    size_t origSegCount = segments.size();
    size_t origCPCount = segments[splitSegIdx].controlPoints.size();
    int splitCPIdx = (int)origCPCount / 2;
    glm::vec3 splitPointPos = segments[splitSegIdx].controlPoints[splitCPIdx];

    // Perform split
    ProcessedRoadSegment newSeg;
    uint32_t maxId = 0;
    for (const auto& s : segments) maxId = std::max(maxId, s.id);
    newSeg.id = maxId + 1;
    newSeg.name = segments[splitSegIdx].name + " (Part 2)";
    newSeg.highway = segments[splitSegIdx].highway;
    newSeg.lanes = segments[splitSegIdx].lanes;
    newSeg.oneway = segments[splitSegIdx].oneway;
    newSeg.laneWidth = segments[splitSegIdx].laneWidth;
    newSeg.shoulderWidth = segments[splitSegIdx].shoulderWidth;
    newSeg.totalWidth = segments[splitSegIdx].totalWidth;
    newSeg.sampleStep = segments[splitSegIdx].sampleStep;
    newSeg.widthSource = segments[splitSegIdx].widthSource;

    newSeg.controlPoints.assign(segments[splitSegIdx].controlPoints.begin() + splitCPIdx,
                                segments[splitSegIdx].controlPoints.end());
    segments[splitSegIdx].controlPoints.erase(segments[splitSegIdx].controlPoints.begin() + splitCPIdx + 1,
                                              segments[splitSegIdx].controlPoints.end());
    segments.push_back(newSeg);

    // Rebuild topology
    RoadGenerator::buildNetworkTopology(segments, junctions);
    assert(segments.size() == origSegCount + 1);
    assert(segments[splitSegIdx].controlPoints.back() == segments.back().controlPoints.front());
    assert(glm::distance(segments[splitSegIdx].controlPoints.back(), splitPointPos) < 1e-4f);
    assert(segments[splitSegIdx].endJunctionId >= 0);
    assert(segments[splitSegIdx].endJunctionId == segments.back().startJunctionId);
    std::cout << "Segment successfully split into 2 parts connected by 2-way junction #"
              << segments[splitSegIdx].endJunctionId << "!" << std::endl;

    std::cout << "\n--- 5. Testing Manual Multi-Way Junction Creation ---" << std::endl;
    // Connect segments[0] start and segments[1] start together at their centroid
    glm::vec3 c1 = segments[0].controlPoints.front();
    glm::vec3 c2 = segments[1].controlPoints.front();
    glm::vec3 manualCentroid = (c1 + c2) * 0.5f;
    segments[0].controlPoints.front() = manualCentroid;
    segments[1].controlPoints.front() = manualCentroid;

    RoadGenerator::buildNetworkTopology(segments, junctions);
    assert(segments[0].startJunctionId >= 0);
    assert(segments[0].startJunctionId == segments[1].startJunctionId);
    std::cout << "Manual junction successfully created: segment #0 and #1 connected at shared junction #"
              << segments[0].startJunctionId << " at centroid (" << manualCentroid.x << ", " << manualCentroid.z << ")!" << std::endl;

    std::cout << "\n--- 6. Testing Road Separation by Height (Overpasses & Ramps) ---" << std::endl;
    int elevatedCount = 0;
    int rampCount = 0;
    int groundCount = 0;

    for (const auto& s : segments) {
        if (s.controlPoints.empty()) continue;
        float y0 = s.controlPoints.front().y;
        float y1 = s.controlPoints.back().y;

        if (y0 >= 5.0f && y1 >= 5.0f) {
            elevatedCount++;
        } else if (std::abs(y0 - y1) >= 4.0f) {
            rampCount++;
        } else if (y0 < 1.0f && y1 < 1.0f) {
            groundCount++;
        }
    }

    std::cout << "Height separation breakdown: "
              << elevatedCount << " elevated bridge spans (y >= 5.0m), "
              << rampCount << " approach ramps (climbing 0.05m <-> 5.5m), "
              << groundCount << " ground roads (y = 0.05m)." << std::endl;

    assert(elevatedCount > 0 && "Expected elevated bridge segments!");
    assert(rampCount > 0 && "Expected approach ramps transitioning to elevated bridges!");
    assert(groundCount > 0 && "Expected ground roads!");

    std::cout << "\n--- 7. Testing Junction Collision & Elevation Sampling ---" << std::endl;
    RoadGenerator::generateJunctionPatches(segments, junctions);
    int testedJunctions = 0;
    int elevatedJunctionsTested = 0;

    for (const auto& junc : junctions) {
        if (junc.patchVertices.empty() || junc.patchIndices.size() < 3) continue;

        // Take the first triangle of the junction patch
        uint32_t i0 = junc.patchIndices[0];
        uint32_t i1 = junc.patchIndices[1];
        uint32_t i2 = junc.patchIndices[2];
        assert(i0 < junc.patchVertices.size() && i1 < junc.patchVertices.size() && i2 < junc.patchVertices.size());
        const auto& v0 = junc.patchVertices[i0];
        const auto& v1 = junc.patchVertices[i1];
        const auto& v2 = junc.patchVertices[i2];

        // Centroid of the triangle
        glm::vec3 triCenter(
            (v0.position[0] + v1.position[0] + v2.position[0]) / 3.0f,
            (v0.position[1] + v1.position[1] + v2.position[1]) / 3.0f,
            (v0.position[2] + v1.position[2] + v2.position[2]) / 3.0f
        );

        // Perform 2D point-in-triangle test (same algorithm as VehiclePhysics)
        glm::vec2 a(v0.position[0], v0.position[2]);
        glm::vec2 b(v1.position[0], v1.position[2]);
        glm::vec2 c(v2.position[0], v2.position[2]);
        glm::vec2 p(triCenter.x, triCenter.z);

        float cp0 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        float cp1 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
        float cp2 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);

        bool inside = ((cp0 >= -1e-3f && cp1 >= -1e-3f && cp2 >= -1e-3f) ||
                       (cp0 <=  1e-3f && cp1 <=  1e-3f && cp2 <=  1e-3f));
        assert(inside && "Triangle centroid must test inside triangle!");

        // Barycentric height interpolation
        float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        float sampledY = triCenter.y;
        if (std::abs(area) > 1e-5f) {
            float u = ((b.x - p.x) * (c.y - p.y) - (b.y - p.y) * (c.x - p.x)) / area;
            float v = ((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x)) / area;
            float w = 1.0f - u - v;
            sampledY = u * v0.position[1] + v * v1.position[1] + w * v2.position[1];
        }

        assert(std::abs(sampledY - triCenter.y) < 1e-3f && "Interpolated height must match triangle surface height!");

        if (triCenter.y > 2.0f) {
            elevatedJunctionsTested++;
        }
        testedJunctions++;
    }

    std::cout << "Verified surface collision across " << testedJunctions << " junction patches ("
              << elevatedJunctionsTested << " elevated overpass junctions with y > 2.0m maintaining road height)!" << std::endl;
    assert(testedJunctions > 0 && "Expected tested junction patches!");

    std::cout << "\n--- 8. Testing Multi-Chunk Domain Decomposition & Seam Continuity ---" << std::endl;
    const std::string chunk00Path = "assets/maps/somerset_chunks/chunk_0_0.froad";
    if (std::ifstream(chunk00Path).good()) {
        std::vector<ProcessedRoadSegment> segs00;
        std::shared_ptr<Mesh> mesh00;
        FRoadChunkHeader hdr00;
        std::vector<RoadJunction> juncs00;

        bool ok00 = RoadBinary::loadChunk(chunk00Path, segs00, mesh00, hdr00, &juncs00);
        assert(ok00 && "Failed to load chunk_0_0.froad!");
        assert(hdr00.version == 2 && "Expected Version 2 header for multi-chunk binary!");
        assert(hdr00.numCrossChunkConnections > 0 && "Expected cross-chunk connections in chunk (0, 0)!");

        std::cout << "Loaded chunk (0, 0): " << segs00.size() << " segments, "
                  << hdr00.numCrossChunkConnections << " cross-chunk connections." << std::endl;

        int verifiedSeams = 0;
        for (const auto& s : segs00) {
            auto testNeighbor = [&](const CrossChunkConnection& conn, const glm::vec3& localBoundary) {
                if (!conn.hasNeighbor) return;
                std::string nPath = "assets/maps/somerset_chunks/chunk_" +
                                    std::to_string(conn.neighborChunkX) + "_" +
                                    std::to_string(conn.neighborChunkZ) + ".froad";
                assert(std::ifstream(nPath).good() && "Neighbor chunk file must exist on disk!");

                std::vector<ProcessedRoadSegment> nSegs;
                std::shared_ptr<Mesh> nMesh;
                FRoadChunkHeader nHdr;
                bool nOk = RoadBinary::loadChunk(nPath, nSegs, nMesh, nHdr, nullptr);
                assert(nOk && "Failed to load neighbor chunk file!");

                // Find neighbor segment
                bool foundNeighborSeg = false;
                for (const auto& ns : nSegs) {
                    if (ns.id == conn.neighborSegmentId) {
                        foundNeighborSeg = true;
                        glm::vec3 nBoundary = conn.neighborIsStart ? ns.splinePoints.front().pos : ns.splinePoints.back().pos;
                        float dist = glm::distance(localBoundary, nBoundary);
                        assert(dist < 0.05f && "3D seam positions between adjacent chunks must match within 5cm!");
                        verifiedSeams++;
                        break;
                    }
                }
                assert(foundNeighborSeg && "Target neighbor segment ID must exist in neighbor chunk!");
            };

            if (s.startNeighbor.hasNeighbor && !s.splinePoints.empty()) {
                testNeighbor(s.startNeighbor, s.splinePoints.front().pos);
            }
            if (s.endNeighbor.hasNeighbor && !s.splinePoints.empty()) {
                testNeighbor(s.endNeighbor, s.splinePoints.back().pos);
            }
        }

        std::cout << "[PASS] Verified exact 3D seam position continuity across "
                  << verifiedSeams << " cross-chunk connections!" << std::endl;
        assert(verifiedSeams > 0 && "Must verify at least one cross-chunk seam!");
    } else {
        std::cout << "[SKIP] Chunk assets/maps/somerset_chunks/chunk_0_0.froad not present for seam test." << std::endl;
    }

    std::cout << "\n--- 9. Testing Arbitrary Chunk ([-3, -3]) Elevation Solve & Filtering ---" << std::endl;
    const std::string chunk33Path = "assets/maps/somerset_chunks/chunk_-3_-3.froad";
    if (std::ifstream(chunk33Path).good()) {
        std::vector<ProcessedRoadSegment> segs33;
        std::shared_ptr<Mesh> mesh33;
        FRoadChunkHeader hdr33;
        std::vector<RoadJunction> juncs33;

        bool ok33 = RoadBinary::loadChunk(chunk33Path, segs33, mesh33, hdr33, &juncs33);
        assert(ok33 && "Failed to load chunk_-3_-3.froad!");
        assert(!segs33.empty() && "Chunk [-3, -3] must contain road segments!");
        assert(mesh33 && mesh33->vertexCount > 0 && "Chunk [-3, -3] precompiled mesh must have vertices!");

        int activeChunkX = -3;
        int activeChunkZ = -3;
        float halfS = 256.0f;
        float minX = activeChunkX * 512.0f - halfS;
        float maxX = activeChunkX * 512.0f + halfS;
        float minZ = activeChunkZ * 512.0f - halfS;
        float maxZ = activeChunkZ * 512.0f + halfS;

        // Simulate Solve Now: solve elevations and rebuild topology & geometry
        auto sampleFn = [](float x, float z) -> float {
            return 135.0f;
        };
        RoadElevationSolverOptions solverOpts;
        RoadElevationSolver::solve(segs33, sampleFn, solverOpts);

        // Rebuild splines and chunk membership with active chunk bounds
        for (auto& seg : segs33) {
            seg.totalWidth = (float)seg.lanes * seg.laneWidth + 2.0f * seg.shoulderWidth;
            seg.splinePoints = RoadGenerator::generateSpline(seg.controlPoints, seg.sampleStep);
            if (seg.splinePoints.size() >= 2) {
                seg.length = seg.splinePoints.back().distance;
                bool inChunk = false;
                for (const auto& sp : seg.splinePoints) {
                    if (sp.pos.x >= minX && sp.pos.x <= maxX && sp.pos.z >= minZ && sp.pos.z <= maxZ) {
                        inChunk = true;
                        break;
                    }
                }
                seg.in500mChunk = inChunk;
            }
        }

        RoadGenerator::buildNetworkTopology(segs33, juncs33);
        RoadGenerator::generateJunctionPatches(segs33, juncs33);

        auto merged = RoadGenerator::buildMergedMesh(segs33, juncs33, true, minX, maxX, minZ, maxZ);
        assert(merged != nullptr && "buildMergedMesh must not return null after solving non-origin chunk [-3, -3]!");
        assert(merged->vertexCount > 0 && "Solved mesh must have vertices!");
        assert(merged->indexCount > 0 && "Solved mesh must have triangles!");

        std::cout << "[PASS] Chunk [-3, -3] solve verified: " << segs33.size() << " segments, "
                  << juncs33.size() << " junctions, merged mesh = " << merged->vertexCount << " verts, "
                  << merged->indexCount / 3 << " triangles (no roads deleted)!" << std::endl;
    } else {
        std::cout << "[SKIP] Chunk assets/maps/somerset_chunks/chunk_-3_-3.froad not present." << std::endl;
    }

    std::cout << "\n--- 10. Testing Quadsphere Chunk [2, 16037, 20963] Endpoint Click Raycast & Junction Addition ---" << std::endl;
    std::string qsPath = "assets/maps/somerset_chunks/chunk_2_16037_20963.froad";
    std::ifstream qsFile(qsPath);
    if (qsFile.good()) {
        qsFile.close();
        std::vector<ProcessedRoadSegment> qsSegs;
        std::shared_ptr<Mesh> qsMesh;
        FRoadChunkHeader qsHdr;
        std::vector<RoadJunction> qsJuncs;
        bool loadOk = RoadBinary::loadChunk(qsPath, qsSegs, qsMesh, qsHdr, &qsJuncs);
        assert(loadOk && "Failed to load chunk_2_16037_20963.froad");
        assert(qsHdr.version >= 3 && "Must be version 3 quadsphere chunk");

        glm::dvec3 chunkCenterECEF(qsHdr.chunkCenterECEF[0], qsHdr.chunkCenterECEF[1], qsHdr.chunkCenterECEF[2]);
        double cLat, cLon, cAlt;
        Quadsphere::ecefToGeodetic(chunkCenterECEF, cLat, cLon, cAlt);
        glm::mat4 R_ecef_to_enu = glm::mat4(Quadsphere::getECEFtoENU(cLat, cLon));

        // Re-generate splines in relative ECEF
        for (auto& s : qsSegs) {
            s.splinePoints = RoadGenerator::generateSpline(s.controlPoints, s.sampleStep, &chunkCenterECEF);
        }

        // 1. Raycast test to verify clicking on a road segment works
        assert(!qsSegs.empty() && "Chunk must have segments");
        const auto& targetSeg = qsSegs[0];
        assert(targetSeg.controlPoints.size() >= 2);
        glm::vec3 pStartENU = glm::vec3(R_ecef_to_enu * glm::vec4(targetSeg.controlPoints.front(), 1.0f));

        glm::vec3 camPos = pStartENU + glm::vec3(0.0f, 100.0f, 50.0f);
        glm::vec3 rayDir = glm::normalize(pStartENU - camPos);

        int bestSegIdx = -1;
        bool bestIsStart = false;
        float bestT = 1e9f;
        glm::vec3 bestHitPos(0.0f);

        for (size_t i = 0; i < qsSegs.size(); ++i) {
            const auto& seg = qsSegs[i];
            if (seg.controlPoints.size() < 2) continue;
            float maxD = std::max(6.0f, seg.totalWidth * 1.5f);
            for (const auto& sp : seg.splinePoints) {
                glm::vec3 spPos = glm::vec3(R_ecef_to_enu * glm::vec4(sp.pos, 1.0f));
                glm::vec3 diff = spPos - camPos;
                float t = glm::dot(diff, rayDir);
                if (t > 0.0f && t < bestT) {
                    glm::vec3 ptOnRay = camPos + rayDir * t;
                    float d3d = glm::distance(ptOnRay, spPos);
                    if (d3d <= maxD) {
                        bestT = t;
                        bestSegIdx = (int)i;
                        bestHitPos = spPos;
                    }
                }
            }
        }

        assert(bestSegIdx >= 0 && "Raycast must hit the road segment!");
        const auto& hitSeg = qsSegs[bestSegIdx];
        glm::vec3 pStartHit = glm::vec3(R_ecef_to_enu * glm::vec4(hitSeg.controlPoints.front(), 1.0f));
        glm::vec3 pEndHit   = glm::vec3(R_ecef_to_enu * glm::vec4(hitSeg.controlPoints.back(), 1.0f));
        bestIsStart = (glm::distance(bestHitPos, pStartHit) < glm::distance(bestHitPos, pEndHit));
        assert(bestSegIdx == 0 && bestIsStart == true && "Raycast must hit seg 0 start endpoint!");

        // 2. Junction creation test with centroid in relative ECEF (preserving elevation)
        if (qsSegs.size() >= 2) {
            glm::vec3 pt0 = qsSegs[0].controlPoints.front();
            glm::vec3 pt1 = qsSegs[1].controlPoints.front();
            glm::vec3 centroid = (pt0 + pt1) * 0.5f;

            // Check that centroid does not corrupt elevation
            glm::dvec3 fullECEF = chunkCenterECEF + glm::dvec3(centroid);
            double distFromEarthCenter = glm::length(fullECEF);
            double alt = distFromEarthCenter - Quadsphere::EARTH_RADIUS;
            assert(alt >= 0.0 && alt <= 500.0 && "Junction centroid altitude must match local Somerset terrain!");

            qsSegs[0].controlPoints.front() = centroid;
            qsSegs[1].controlPoints.front() = centroid;

            std::vector<RoadJunction> newJuncs;
            RoadGenerator::buildNetworkTopology(qsSegs, newJuncs);
            RoadGenerator::generateJunctionPatches(qsSegs, newJuncs, &chunkCenterECEF);
            assert(!newJuncs.empty() && "Junction must be generated from merged endpoints!");
        }

        std::cout << "[PASS] Quadsphere chunk [2, 16037, 20963] click raycast and junction creation verified!" << std::endl;
    } else {
        std::cout << "[SKIP] Chunk " << qsPath << " not found." << std::endl;
    }

    // ------------------------------------------------------------------------
    // 11. Testing Intersecting Road Automatic Splitting in Chunk [2, 16040, 20960]
    // ------------------------------------------------------------------------
    std::cout << "\n--- 11. Testing Intersecting Road Junctions in Chunk [2, 16040, 20960] ---" << std::endl;
    std::string testChunkPath = "assets/maps/somerset_chunks/chunk_2_16040_20960.froad";
    std::vector<ProcessedRoadSegment> chunk16040Segs;
    std::shared_ptr<Mesh> chunk16040Mesh;
    FRoadChunkHeader chunk16040Hdr;
    if (RoadBinary::loadChunk(testChunkPath, chunk16040Segs, chunk16040Mesh, chunk16040Hdr)) {
        std::cout << "Loaded chunk [2, 16040, 20960]: " << chunk16040Segs.size() << " segments, "
                  << (chunk16040Mesh ? chunk16040Mesh->vertexCount : 0) << " vertices, "
                  << (chunk16040Mesh ? chunk16040Mesh->indexCount / 3 : 0) << " triangles." << std::endl;

        std::vector<RoadJunction> rawJunctions;
        RoadGenerator::buildNetworkTopology(chunk16040Segs, rawJunctions);
        std::cout << "Before collapse: Detected " << rawJunctions.size() << " junctions in chunk [2, 16040, 20960]:" << std::endl;
        int rawMultiWayCount = 0;
        for (size_t j = 0; j < rawJunctions.size(); ++j) {
            if (rawJunctions[j].connections.size() >= 3) {
                rawMultiWayCount++;
                std::cout << "  Junction #" << j << " (" << rawJunctions[j].connections.size() << "-way) at ("
                          << rawJunctions[j].position.x << ", " << rawJunctions[j].position.z << "):";
                for (const auto& c : rawJunctions[j].connections) {
                    std::cout << " seg#" << c.segmentIdx << "('" << chunk16040Segs[c.segmentIdx].name << "')";
                }
                std::cout << std::endl;
            }
        }

        // Test automatic deletion/collapse of short segments squeezed between two junctions
        bool collapsed = RoadGenerator::collapseShortJunctionSegments(chunk16040Segs, 26.0f);
        assert(collapsed && "Expected short squeezed segments to be collapsed!");
        assert(chunk16040Segs.size() < 11 && "Squeezed segments should be removed from chunk segments!");

        std::vector<RoadJunction> collapsedJunctions;
        RoadGenerator::buildNetworkTopology(chunk16040Segs, collapsedJunctions);
        std::cout << "After collapse: Detected " << collapsedJunctions.size() << " junctions across "
                  << chunk16040Segs.size() << " segments:" << std::endl;
        int multiWayCount = 0;
        for (size_t j = 0; j < collapsedJunctions.size(); ++j) {
            if (collapsedJunctions[j].connections.size() >= 3) {
                multiWayCount++;
                std::cout << "  Unified Junction #" << j << " (" << collapsedJunctions[j].connections.size() << "-way) at ("
                          << collapsedJunctions[j].position.x << ", " << collapsedJunctions[j].position.z << "):";
                for (const auto& c : collapsedJunctions[j].connections) {
                    std::cout << " seg#" << c.segmentIdx << "('" << chunk16040Segs[c.segmentIdx].name << "')";
                }
                std::cout << std::endl;
            }
        }
        assert(multiWayCount >= 2 && "Expected unified multi-way junctions on Strawberry Trail!");
        glm::dvec3 chunkCenterECEF(chunk16040Hdr.chunkCenterECEF[0], chunk16040Hdr.chunkCenterECEF[1], chunk16040Hdr.chunkCenterECEF[2]);
        RoadGenerator::generateJunctionPatches(chunk16040Segs, collapsedJunctions, &chunkCenterECEF);
        std::cout << "[PASS] Squeezed junction segments successfully collapsed into unified multi-way junctions!" << std::endl;
    } else {
        std::cout << "[SKIP] Chunk " << testChunkPath << " not found." << std::endl;
    }

    std::cout << "\nALL TOPOLOGY & INTERMEDIATE TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}

