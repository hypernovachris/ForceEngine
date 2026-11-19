#include "Road/RoadGenerator.h"
#include "Terrain/QuadsphereMath.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <tuple>
#include <utility>

static glm::dvec2 localPosToGPS(const glm::vec3& p) {
    const double cLat = 40.592676;
    const double cLon = -74.515211;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);
    double lat = cLat - ((double)p.z / m_per_lat);
    double lon = cLon + ((double)p.x / m_per_lon);
    return glm::dvec2(lon, lat);
}

static void ensureValidGPS(RawRoadFeature& feat) {
    if (feat.gpsPoints.size() != feat.rawPoints.size()) {
        feat.gpsPoints.clear();
        feat.gpsPoints.reserve(feat.rawPoints.size());
        for (const auto& p : feat.rawPoints) {
            feat.gpsPoints.push_back(localPosToGPS(p));
        }
    } else {
        for (size_t i = 0; i < feat.gpsPoints.size(); ++i) {
            if (std::abs(feat.gpsPoints[i].x) < 1e-4 && std::abs(feat.gpsPoints[i].y) < 1e-4) {
                feat.gpsPoints[i] = localPosToGPS(feat.rawPoints[i]);
            }
        }
    }
}

std::vector<RoadSplinePoint> RoadGenerator::generateSpline(const std::vector<glm::vec3>& rawPoints,
                                                           float sampleStep,
                                                           const glm::dvec3* optChunkCenterECEF) {
    std::vector<RoadSplinePoint> spline;
    if (rawPoints.size() < 2) return spline;

    auto makePoint = [&](const glm::vec3& pos, const glm::vec3& tangent, float dist) -> RoadSplinePoint {
        RoadSplinePoint sp;
        sp.pos = pos;
        sp.tangent = tangent;
        sp.distance = dist;
        if (optChunkCenterECEF) {
            sp.posECEF = *optChunkCenterECEF + glm::dvec3(pos);
            sp.normal = (glm::length(sp.posECEF) > 1e-3) ? glm::vec3(glm::normalize(sp.posECEF)) : glm::vec3(0.0f, 1.0f, 0.0f);
        } else {
            sp.posECEF = glm::dvec3(pos);
            sp.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        return sp;
    };

    // 1. Cull duplicate / micro-distance intermediate control points (< 0.75m)
    // Always preserve first and last control points to guarantee exact junction connectivity
    std::vector<glm::vec3> pts;
    pts.reserve(rawPoints.size());
    for (size_t i = 0; i < rawPoints.size(); ++i) {
        if (i == 0 || i == rawPoints.size() - 1) {
            pts.push_back(rawPoints[i]);
        } else {
            float d = glm::distance(glm::vec2(rawPoints[i].x, rawPoints[i].z),
                                    glm::vec2(pts.back().x, pts.back().z));
            if (d >= 0.75f) {
                pts.push_back(rawPoints[i]);
            }
        }
    }
    if (pts.size() < 2) pts = rawPoints;

    size_t N = pts.size();
    if (N == 2) {
        glm::vec3 p0 = pts[0];
        glm::vec3 p1 = pts[1];
        float segLen = glm::length(p1 - p0);
        int steps = std::max(2, (int)std::ceil(segLen / sampleStep));
        glm::vec3 tangent = (segLen > 1e-4f) ? glm::normalize(p1 - p0) : glm::vec3(0.0f, 0.0f, 1.0f);

        float accDist = 0.0f;
        glm::vec3 lastPos = p0;
        for (int s = 0; s <= steps; ++s) {
            float t = (float)s / (float)steps;
            glm::vec3 pos = glm::mix(p0, p1, t);
            if (s > 0) accDist += glm::length(pos - lastPos);
            lastPos = pos;
            spline.push_back(makePoint(pos, tangent, accDist));
        }
        return spline;
    }

    // 2. Centripetal Catmull-Rom Spline Interpolation (alpha = 0.5)
    // Mathematically guaranteed to prevent loops, cusps, overshoots, and self-intersections
    std::vector<glm::vec3> sampledPositions;

    for (size_t i = 0; i < N - 1; ++i) {
        glm::vec3 p1 = pts[i];
        glm::vec3 p2 = pts[i + 1];
        glm::vec3 p0 = (i > 0) ? pts[i - 1] : p1 - (p2 - p1);
        glm::vec3 p3 = (i + 2 < N) ? pts[i + 2] : p2 + (p2 - p1);

        auto getKnot = [](const glm::vec3& a, const glm::vec3& b) -> float {
            float d = glm::distance(glm::vec2(a.x, a.z), glm::vec2(b.x, b.z));
            return std::max(1e-3f, std::sqrt(d)); // alpha = 0.5
        };

        float t0 = 0.0f;
        float t1 = t0 + getKnot(p0, p1);
        float t2 = t1 + getKnot(p1, p2);
        float t3 = t2 + getKnot(p2, p3);

        float segLen = glm::length(p2 - p1);
        int steps = std::max(2, (int)std::ceil(segLen / sampleStep));

        auto lerpPt = [](const glm::vec3& pA, const glm::vec3& pB, float tA, float tB, float tVal) {
            float f = (tVal - tA) / (tB - tA);
            return glm::mix(pA, pB, f);
        };

        for (int s = 0; s < steps; ++s) {
            float u = (float)s / (float)steps;
            float t = t1 + u * (t2 - t1);

            glm::vec3 a1 = lerpPt(p0, p1, t0, t1, t);
            glm::vec3 a2 = lerpPt(p1, p2, t1, t2, t);
            glm::vec3 a3 = lerpPt(p2, p3, t2, t3, t);

            glm::vec3 b1 = lerpPt(a1, a2, t0, t2, t);
            glm::vec3 b2 = lerpPt(a2, a3, t1, t3, t);

            glm::vec3 pos = lerpPt(b1, b2, t1, t2, t);
            sampledPositions.push_back(pos);
        }
    }
    sampledPositions.push_back(pts.back());

    // 3. Compute smooth monotonic tangents and arc-length distances along sampled curve
    float accDist = 0.0f;
    for (size_t k = 0; k < sampledPositions.size(); ++k) {
        if (k > 0) {
            accDist += glm::length(sampledPositions[k] - sampledPositions[k - 1]);
        }

        glm::vec3 dir;
        if (k == 0) {
            dir = sampledPositions[1] - sampledPositions[0];
        } else if (k == sampledPositions.size() - 1) {
            dir = sampledPositions.back() - sampledPositions[k - 1];
        } else {
            dir = sampledPositions[k + 1] - sampledPositions[k - 1];
        }
        float len = glm::length(dir);
        glm::vec3 tangent = (len > 1e-4f) ? (dir / len) : glm::vec3(0.0f, 0.0f, 1.0f);

        spline.push_back(makePoint(sampledPositions[k], tangent, accDist));
    }

    return spline;
}

void RoadGenerator::deduceRoadParameters(const RawRoadFeature& feat,
                                         float& outLaneWidth,
                                         float& outShoulderWidth,
                                         int& outLanes,
                                         bool& outOneWay,
                                         std::string& outWidthSource) {
    // 1. Lanes
    if (feat.hasExplicitLanes && feat.lanes > 0) {
        outLanes = feat.lanes;
    } else {
        if (feat.highway == "motorway") outLanes = 3;
        else if (feat.highway == "trunk") outLanes = 2;
        else if (feat.highway == "trunk_link" || feat.highway == "motorway_link") outLanes = 1;
        else if (feat.highway == "residential") outLanes = 2;
        else if (feat.highway == "tertiary") outLanes = 2;
        else outLanes = 2;
    }

    // 2. One-Way
    outOneWay = feat.oneway;

    // Check highway classification standards
    bool isInterstate = (feat.highway == "motorway" || feat.highway == "motorway_link" ||
                         feat.ref.find("I ") != std::string::npos || feat.ref.find("I-") != std::string::npos);
    bool isTrunk = (feat.highway == "trunk" || feat.highway == "trunk_link" ||
                    feat.ref.find("US ") != std::string::npos);
    bool isRamp = (feat.highway == "trunk_link" || feat.highway == "motorway_link");
    bool isPrimary = (feat.highway == "primary" || feat.highway == "secondary" ||
                      feat.ref.find("NJ ") != std::string::npos || feat.ref.find("Route") != std::string::npos);
    bool isTertiary = (feat.highway == "tertiary" || feat.highway == "tertiary_link");
    bool isResidential = (feat.highway == "residential" || feat.highway == "unclassified" || feat.highway == "living_street");

    // 3. Lane Width (Data first, then standards)
    if (feat.parsedLaneWidth > 0.0f) {
        outLaneWidth = feat.parsedLaneWidth;
        outWidthSource = "OSM Tag (lane_width)";
    } else if (feat.parsedWidth > 0.0f) {
        float estShoulder = (feat.parsedShoulderWidth >= 0.0f) ? feat.parsedShoulderWidth : 1.5f;
        outLaneWidth = std::max(2.6f, (feat.parsedWidth - 2.0f * estShoulder) / (float)outLanes);
        outWidthSource = "OSM Tag (width)";
    } else {
        // AASHTO / State DOT Highway Standards
        if (isInterstate) {
            outLaneWidth = 3.66f; // AASHTO Interstate standard: 12.0 feet
            outWidthSource = "AASHTO Interstate Standard (12.0 ft / 3.66m)";
        } else if (isRamp) {
            outLaneWidth = 4.27f; // AASHTO interchange ramp standard: 14.0 feet
            outWidthSource = "AASHTO Ramp Standard (14.0 ft / 4.27m)";
        } else if (isTrunk) {
            outLaneWidth = 3.66f; // AASHTO Arterial standard: 12.0 feet
            outWidthSource = "AASHTO Arterial Standard (12.0 ft / 3.66m)";
        } else if (isPrimary) {
            outLaneWidth = 3.50f; // State primary standard: 11.5 feet
            outWidthSource = "State Highway Standard (11.5 ft / 3.50m)";
        } else if (isTertiary) {
            outLaneWidth = 3.20f; // Collector standard: 10.5 feet
            outWidthSource = "Collector Standard (10.5 ft / 3.20m)";
        } else if (isResidential) {
            outLaneWidth = 3.00f; // Local residential: 9.8 feet
            outWidthSource = "Residential Local Standard (9.8 ft / 3.00m)";
        } else {
            outLaneWidth = 3.50f;
            outWidthSource = "Standard Default (3.50m)";
        }
    }

    // 4. Shoulder Width (Data first, then standards)
    if (feat.parsedShoulderWidth >= 0.0f) {
        outShoulderWidth = feat.parsedShoulderWidth;
        outWidthSource += " + OSM Shoulder";
    } else if (feat.shoulderTag == "no" || feat.shoulderTag == "none") {
        outShoulderWidth = 0.0f;
        outWidthSource += " (no shoulder)";
    } else {
        if (isInterstate) {
            outShoulderWidth = 3.05f; // 10.0 ft paved shoulder
        } else if (isRamp) {
            outShoulderWidth = 1.50f; // 5.0 ft paved shoulder
        } else if (isTrunk) {
            outShoulderWidth = 2.44f; // 8.0 ft paved shoulder
        } else if (isPrimary) {
            outShoulderWidth = 1.83f; // 6.0 ft paved shoulder
        } else if (isTertiary) {
            outShoulderWidth = 1.00f; // 3.3 ft shoulder
        } else if (isResidential) {
            outShoulderWidth = 0.00f; // Curb only, 0 paved shoulder
        } else {
            outShoulderWidth = 1.50f;
        }

        if (feat.shoulderTag == "right" || feat.shoulderTag == "left") {
            outShoulderWidth *= 0.5f; // One side only
        }
    }
}

void RoadGenerator::generateRibbonGeometry(const std::vector<RoadSplinePoint>& splinePoints,
                                          float totalWidth,
                                          int lanes,
                                          bool oneway,
                                          float shoulderWidth,
                                          std::vector<Vertex>& outVertices,
                                          std::vector<uint32_t>& outIndices) {
    if (splinePoints.size() < 2) return;

    uint32_t baseVertex = (uint32_t)outVertices.size();
    float halfW = totalWidth * 0.5f;
    float shoulderRatio = (totalWidth > 0.001f) ? std::clamp(shoulderWidth / totalWidth, 0.0f, 0.35f) : 0.05f;

    // 3D Curb Profile dimensions (Civil Engineering Standards)
    // 30cm wide curb extending 15cm above road deck and 50cm below into ground
    const float curbW = 0.30f;       // 30cm wide
    const float curbH_top = 0.15f;   // 15cm above road
    const float curbH_bot = 0.50f;   // 50cm below road

    auto makeVertex = [](const glm::vec3& pos, const glm::vec3& norm, float u, float v, const glm::vec3& attr) -> Vertex {
        Vertex vert = {};
        vert.position[0] = pos.x; vert.position[1] = pos.y; vert.position[2] = pos.z;
        vert.normal[0]   = norm.x; vert.normal[1]   = norm.y; vert.normal[2]   = norm.z;
        vert.texCoords[0] = u;     vert.texCoords[1] = v;
        vert.tangent[0]  = attr.x; vert.tangent[1]  = attr.y; vert.tangent[2]  = attr.z;
        return vert;
    };

    glm::vec3 curbAttr(-1.0f, 0.0f, 0.0f);
    glm::vec3 roadAttr((float)lanes, oneway ? 1.0f : 0.0f, shoulderRatio);

    size_t K = splinePoints.size();
    for (size_t k = 0; k < K; ++k) {
        const auto& sp = splinePoints[k];
        glm::vec3 P = sp.pos;
        glm::vec3 N = (glm::length(sp.normal) > 0.5f) ? glm::normalize(sp.normal) : glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 T = sp.tangent;
        if (glm::length(T) < 1e-4f) {
            if (k + 1 < K) T = splinePoints[k + 1].pos - sp.pos;
            else if (k > 0) T = sp.pos - splinePoints[k - 1].pos;
        }
        T = T - N * glm::dot(T, N);
        if (glm::length(T) > 1e-4f) T = glm::normalize(T);
        else T = (std::abs(N.y) < 0.9f) ? glm::normalize(glm::cross(N, glm::vec3(0.0f, 1.0f, 0.0f))) : glm::vec3(1.0f, 0.0f, 0.0f);

        glm::vec3 R = glm::normalize(glm::cross(T, N));
        float v = sp.distance;

        // Road deck edges (at road surface level)
        glm::vec3 p_roadL = P - R * halfW;
        glm::vec3 p_roadR = P + R * halfW;

        // Left curb profile (outer bottom -> outer top -> inner top -> road left)
        glm::vec3 p_curbL_bot = (P - R * (halfW + curbW)) - N * curbH_bot;
        glm::vec3 p_curbL_out = (P - R * (halfW + curbW)) + N * curbH_top;
        glm::vec3 p_curbL_in  = p_roadL + N * curbH_top;

        // Right curb profile (road right -> inner top -> outer top -> outer bottom)
        glm::vec3 p_curbR_in  = p_roadR + N * curbH_top;
        glm::vec3 p_curbR_out = (P + R * (halfW + curbW)) + N * curbH_top;
        glm::vec3 p_curbR_bot = (P + R * (halfW + curbW)) - N * curbH_bot;

        // 7 distinct quad strips across the road corridor:
        // Strip 0: Left Skirt (drop face facing left -R)
        outVertices.push_back(makeVertex(p_curbL_bot, -R, 0.0f, v, curbAttr));
        outVertices.push_back(makeVertex(p_curbL_out, -R, 0.0f, v, curbAttr));

        // Strip 1: Left Curb Top (horizontal tread facing up +N)
        outVertices.push_back(makeVertex(p_curbL_out, N, 0.5f, v, curbAttr));
        outVertices.push_back(makeVertex(p_curbL_in,  N, 0.5f, v, curbAttr));

        // Strip 2: Left Curb Riser (inner curb face facing road +R)
        outVertices.push_back(makeVertex(p_roadL,    R, 1.0f, v, curbAttr));
        outVertices.push_back(makeVertex(p_curbL_in, R, 1.0f, v, curbAttr));

        // Strip 3: Road Deck (carriageway + shoulders facing up +N)
        outVertices.push_back(makeVertex(p_roadL, N, 0.0f, v, roadAttr));
        outVertices.push_back(makeVertex(p_roadR, N, 1.0f, v, roadAttr));

        // Strip 4: Right Curb Riser (inner curb face facing road -R)
        outVertices.push_back(makeVertex(p_roadR,    -R, 1.0f, v, curbAttr));
        outVertices.push_back(makeVertex(p_curbR_in, -R, 1.0f, v, curbAttr));

        // Strip 5: Right Curb Top (horizontal tread facing up +N)
        outVertices.push_back(makeVertex(p_curbR_in,  N, 0.5f, v, curbAttr));
        outVertices.push_back(makeVertex(p_curbR_out, N, 0.5f, v, curbAttr));

        // Strip 6: Right Skirt (drop face facing right +R)
        outVertices.push_back(makeVertex(p_curbR_bot, R, 0.0f, v, curbAttr));
        outVertices.push_back(makeVertex(p_curbR_out, R, 0.0f, v, curbAttr));
    }

    // Connect quads for all 7 strips along the corridor
    for (size_t k = 0; k < K - 1; ++k) {
        for (int s = 0; s < 7; ++s) {
            uint32_t a0 = baseVertex + (uint32_t)(k * 14 + s * 2 + 0);
            uint32_t b0 = baseVertex + (uint32_t)(k * 14 + s * 2 + 1);
            uint32_t a1 = baseVertex + (uint32_t)((k + 1) * 14 + s * 2 + 0);
            uint32_t b1 = baseVertex + (uint32_t)((k + 1) * 14 + s * 2 + 1);

            glm::vec3 pa0(outVertices[a0].position[0], outVertices[a0].position[1], outVertices[a0].position[2]);
            glm::vec3 pb0(outVertices[b0].position[0], outVertices[b0].position[1], outVertices[b0].position[2]);
            glm::vec3 pa1(outVertices[a1].position[0], outVertices[a1].position[1], outVertices[a1].position[2]);
            glm::vec3 pb1(outVertices[b1].position[0], outVertices[b1].position[1], outVertices[b1].position[2]);

            glm::vec3 triNorm = glm::cross(pa1 - pa0, pb1 - pa0);
            glm::vec3 desiredNorm(outVertices[a0].normal[0], outVertices[a0].normal[1], outVertices[a0].normal[2]);

            if (glm::dot(triNorm, desiredNorm) >= 0.0f) {
                outIndices.push_back(a0);
                outIndices.push_back(a1);
                outIndices.push_back(b1);

                outIndices.push_back(b1);
                outIndices.push_back(b0);
                outIndices.push_back(a0);
            } else {
                outIndices.push_back(a0);
                outIndices.push_back(b1);
                outIndices.push_back(a1);

                outIndices.push_back(b1);
                outIndices.push_back(a0);
                outIndices.push_back(b0);
            }
        }
    }

    // Helper to generate watertight vertical end cap quads on open curb ends
    auto addCapQuad = [&](const glm::vec3& p0, const glm::vec3& p1,
                          const glm::vec3& p2, const glm::vec3& p3,
                          const glm::vec3& norm, float v) {
        uint32_t i0 = (uint32_t)outVertices.size();
        outVertices.push_back(makeVertex(p0, norm, 0.0f, v, curbAttr));
        outVertices.push_back(makeVertex(p1, norm, 0.0f, v, curbAttr));
        outVertices.push_back(makeVertex(p2, norm, 0.5f, v, curbAttr));
        outVertices.push_back(makeVertex(p3, norm, 1.0f, v, curbAttr));

        uint32_t i1 = i0 + 1;
        uint32_t i2 = i0 + 2;
        uint32_t i3 = i0 + 3;

        glm::vec3 nA = glm::cross(p1 - p0, p2 - p0);
        if (glm::dot(nA, norm) >= 0.0f) {
            outIndices.push_back(i0); outIndices.push_back(i1); outIndices.push_back(i2);
            outIndices.push_back(i2); outIndices.push_back(i3); outIndices.push_back(i0);
        } else {
            outIndices.push_back(i0); outIndices.push_back(i2); outIndices.push_back(i1);
            outIndices.push_back(i2); outIndices.push_back(i0); outIndices.push_back(i3);
        }
    };

    // Watertight end caps at road start (k = 0, normal = -T)
    {
        const auto& sp0 = splinePoints.front();
        glm::vec3 T0 = sp0.tangent;
        if (glm::length(T0) < 1e-4f) T0 = splinePoints[1].pos - sp0.pos;
        T0.y = 0.0f;
        if (glm::length(T0) > 1e-4f) T0 = glm::normalize(T0);
        else T0 = glm::vec3(0.0f, 0.0f, 1.0f);

        glm::vec3 N(0.0f, 1.0f, 0.0f);
        glm::vec3 R0 = glm::normalize(glm::cross(T0, N));
        glm::vec3 P0 = sp0.pos;

        glm::vec3 p_roadL0     = P0 - R0 * halfW;
        glm::vec3 p_curbL_bot0 = (P0 - R0 * (halfW + curbW)) - N * curbH_bot;
        glm::vec3 p_curbL_out0 = (P0 - R0 * (halfW + curbW)) + N * curbH_top;
        glm::vec3 p_curbL_in0  = p_roadL0 + N * curbH_top;

        glm::vec3 p_roadR0     = P0 + R0 * halfW;
        glm::vec3 p_curbR_bot0 = (P0 + R0 * (halfW + curbW)) - N * curbH_bot;
        glm::vec3 p_curbR_out0 = (P0 + R0 * (halfW + curbW)) + N * curbH_top;
        glm::vec3 p_curbR_in0  = p_roadR0 + N * curbH_top;

        addCapQuad(p_curbL_bot0, p_curbL_out0, p_curbL_in0, p_roadL0, -T0, sp0.distance);
        addCapQuad(p_curbR_bot0, p_curbR_out0, p_curbR_in0, p_roadR0, -T0, sp0.distance);
    }

    // Watertight end caps at road end (k = K - 1, normal = +T)
    {
        const auto& spEnd = splinePoints.back();
        glm::vec3 Tend = spEnd.tangent;
        if (glm::length(Tend) < 1e-4f) Tend = spEnd.pos - splinePoints[K - 2].pos;
        Tend.y = 0.0f;
        if (glm::length(Tend) > 1e-4f) Tend = glm::normalize(Tend);
        else Tend = glm::vec3(0.0f, 0.0f, 1.0f);

        glm::vec3 N(0.0f, 1.0f, 0.0f);
        glm::vec3 Rend = glm::normalize(glm::cross(Tend, N));
        glm::vec3 Pend = spEnd.pos;

        glm::vec3 p_roadLend     = Pend - Rend * halfW;
        glm::vec3 p_curbL_botEnd = (Pend - Rend * (halfW + curbW)) - N * curbH_bot;
        glm::vec3 p_curbL_outEnd = (Pend - Rend * (halfW + curbW)) + N * curbH_top;
        glm::vec3 p_curbL_inEnd  = p_roadLend + N * curbH_top;

        glm::vec3 p_roadRend     = Pend + Rend * halfW;
        glm::vec3 p_curbR_botEnd = (Pend + Rend * (halfW + curbW)) - N * curbH_bot;
        glm::vec3 p_curbR_outEnd = (Pend + Rend * (halfW + curbW)) + N * curbH_top;
        glm::vec3 p_curbR_inEnd  = p_roadRend + N * curbH_top;

        addCapQuad(p_curbL_botEnd, p_curbL_outEnd, p_curbL_inEnd, p_roadLend, Tend, spEnd.distance);
        addCapQuad(p_curbR_botEnd, p_curbR_outEnd, p_curbR_inEnd, p_roadRend, Tend, spEnd.distance);
    }
}

void RoadGenerator::clipRibbonAgainstJunctions(std::vector<Vertex>& vertices,
                                               const ProcessedRoadSegment& seg,
                                               const std::vector<RoadJunction>& junctions,
                                               const std::vector<ProcessedRoadSegment>& allSegments) {
    // Ribbons terminate cleanly at their setback caps calculated by generateJunctionPatches().
    // Symmetrical clearance setbacks prevent ribbon overlap at diverging junctions,
    // avoiding non-affine quad vertex distortion along curved road corridors.
    (void)vertices;
    (void)seg;
    (void)junctions;
    (void)allSegments;
}

void RoadGenerator::stitchCollinearFeatures(std::vector<RawRoadFeature>& features) {
    const float tol = 0.50f; // 50cm tolerance for matching coordinates

    auto getCellKey = [](float x, float z) -> int64_t {
        int64_t gx = static_cast<int64_t>(std::floor(x));
        int64_t gz = static_cast<int64_t>(std::floor(z));
        return (gx << 32) ^ (gz & 0xFFFFFFFFLL);
    };

    auto getDir = [](const RawRoadFeature& f, bool fromStart) -> glm::vec2 {
        if (f.rawPoints.size() < 2) return glm::vec2(0.0f);
        if (fromStart) {
            glm::vec2 d(f.rawPoints[1].x - f.rawPoints[0].x, f.rawPoints[1].z - f.rawPoints[0].z);
            float l = glm::length(d);
            return (l > 1e-4f) ? (d / l) : glm::vec2(0.0f);
        } else {
            size_t n = f.rawPoints.size();
            glm::vec2 d(f.rawPoints[n-1].x - f.rawPoints[n-2].x, f.rawPoints[n-1].z - f.rawPoints[n-2].z);
            float l = glm::length(d);
            return (l > 1e-4f) ? (d / l) : glm::vec2(0.0f);
        }
    };

    struct EndpointRef {
        int featIdx;
        bool isStart;
    };

    bool changed = true;
    int pass = 0;
    while (changed && pass < 50) {
        changed = false;
        pass++;

        std::unordered_map<int64_t, std::vector<EndpointRef>> grid;
        grid.reserve(features.size() * 2);
        for (int i = 0; i < (int)features.size(); ++i) {
            if (features[i].rawPoints.size() < 2) continue;
            grid[getCellKey(features[i].rawPoints.front().x, features[i].rawPoints.front().z)].push_back({ i, true });
            grid[getCellKey(features[i].rawPoints.back().x, features[i].rawPoints.back().z)].push_back({ i, false });
        }

        std::vector<bool> removed(features.size(), false);

        for (int i = 0; i < (int)features.size(); ++i) {
            if (removed[i] || features[i].rawPoints.size() < 2) continue;

            glm::vec3 ptEnd = features[i].rawPoints.back();
            int64_t bgx = static_cast<int64_t>(std::floor(ptEnd.x));
            int64_t bgz = static_cast<int64_t>(std::floor(ptEnd.z));

            std::vector<EndpointRef> matches;
            for (int64_t dx = -1; dx <= 1; ++dx) {
                for (int64_t dz = -1; dz <= 1; ++dz) {
                    int64_t k = ((bgx + dx) << 32) ^ ((bgz + dz) & 0xFFFFFFFFLL);
                    auto it = grid.find(k);
                    if (it != grid.end()) {
                        for (const auto& ep : it->second) {
                            if (removed[ep.featIdx]) continue;
                            const auto& otherPt = ep.isStart ? features[ep.featIdx].rawPoints.front() : features[ep.featIdx].rawPoints.back();
                            if (glm::distance(glm::vec2(ptEnd.x, ptEnd.z), glm::vec2(otherPt.x, otherPt.z)) < tol &&
                                std::abs(ptEnd.y - otherPt.y) < 1.0f) {
                                matches.push_back(ep);
                            }
                        }
                    }
                }
            }

            if (matches.size() == 2) {
                int j = -1;
                bool jIsStart = false;
                for (const auto& m : matches) {
                    if (m.featIdx != i) {
                        j = m.featIdx;
                        jIsStart = m.isStart;
                        break;
                    }
                }

                if (j >= 0 && !removed[j] && jIsStart) {
                    auto& f0 = features[i];
                    auto& f1 = features[j];

                    if (f0.highway == f1.highway &&
                        f0.lanes == f1.lanes &&
                        f0.oneway == f1.oneway &&
                        f0.layer == f1.layer &&
                        f0.isBridge == f1.isBridge &&
                        f0.isTunnel == f1.isTunnel) {

                        glm::vec2 d0 = getDir(f0, false);
                        glm::vec2 d1 = getDir(f1, true);
                        if (glm::dot(d0, d1) >= 0.5f) { // Collinear within 60 degrees
                            if (f0.name.empty() && !f1.name.empty()) f0.name = f1.name;
                            if (f0.ref.empty() && !f1.ref.empty()) f0.ref = f1.ref;
                            f0.rawPoints.insert(f0.rawPoints.end(), f1.rawPoints.begin() + 1, f1.rawPoints.end());
                            removed[j] = true;
                            changed = true;
                        }
                    }
                }
            }
        }

        if (changed) {
            std::vector<RawRoadFeature> compacted;
            compacted.reserve(features.size());
            for (size_t k = 0; k < features.size(); ++k) {
                if (!removed[k]) compacted.push_back(std::move(features[k]));
            }
            features = std::move(compacted);
        }
    }
}

void RoadGenerator::splitIntersectingFeatures(std::vector<RawRoadFeature>& features, float snapDistance, float minJunctionDistance) {
    if (features.empty()) return;

    auto sameGrade = [](const RawRoadFeature& a, const RawRoadFeature& b) -> bool {
        if (a.layer != b.layer) return false;
        if (a.isBridge != b.isBridge) return false;
        if (a.isTunnel != b.isTunnel) return false;
        if (std::abs(a.elevation - b.elevation) > 2.0f) return false;
        return true;
    };

    auto cross2D = [](const glm::vec2& a, const glm::vec2& b) -> float {
        return a.x * b.y - a.y * b.x;
    };

    struct SplitPoint {
        int segIdx = 0;       // Segment index i in [0, rawPoints.size() - 2]
        float t = 0.0f;       // Parameter along segment [0.0f, 1.0f]
        glm::vec3 pos{0.0f};  // 3D position
        glm::dvec2 gps{0.0};  // WGS84 (lon, lat)

        float key() const {
            return (float)segIdx + t;
        }
    };

    std::vector<std::vector<SplitPoint>> featSplits(features.size());

    // 1. Build 2D spatial hash grid of all line segments
    const float cellSize = 64.0f;
    auto getCellCoord = [cellSize](float val) -> int {
        return static_cast<int>(std::floor(val / cellSize));
    };
    auto makeCellKey = [](int cx, int cz) -> int64_t {
        return ((int64_t)cx << 32) ^ ((int64_t)cz & 0xFFFFFFFFLL);
    };

    struct IndexedSeg {
        int featIdx;
        int segIdx;
    };

    std::unordered_map<int64_t, std::vector<IndexedSeg>> grid;
    grid.reserve(features.size() * 8);

    for (int f = 0; f < (int)features.size(); ++f) {
        const auto& feat = features[f];
        if (feat.rawPoints.size() < 2) continue;

        for (int s = 0; s + 1 < (int)feat.rawPoints.size(); ++s) {
            const auto& p0 = feat.rawPoints[s];
            const auto& p1 = feat.rawPoints[s + 1];

            int minCX = getCellCoord(std::min(p0.x, p1.x) - snapDistance);
            int maxCX = getCellCoord(std::max(p0.x, p1.x) + snapDistance);
            int minCZ = getCellCoord(std::min(p0.z, p1.z) - snapDistance);
            int maxCZ = getCellCoord(std::max(p0.z, p1.z) + snapDistance);

            for (int cx = minCX; cx <= maxCX; ++cx) {
                for (int cz = minCZ; cz <= maxCZ; ++cz) {
                    grid[makeCellKey(cx, cz)].push_back({ f, s });
                }
            }
        }
    }

    int tJunctionsAdded = 0;
    int crossingsAdded = 0;

    // 2. Detect endpoint-to-interior touches and T-junctions
    for (int b = 0; b < (int)features.size(); ++b) {
        auto& featB = features[b];
        if (featB.rawPoints.size() < 2) continue;

        for (int endp = 0; endp < 2; ++endp) {
            bool isStart = (endp == 0);
            glm::vec3 ep = isStart ? featB.rawPoints.front() : featB.rawPoints.back();
            glm::dvec2 epGPS = (!featB.gpsPoints.empty())
                ? (isStart ? featB.gpsPoints.front() : featB.gpsPoints.back())
                : localPosToGPS(ep);
            if (std::abs(epGPS.x) < 1e-4 && std::abs(epGPS.y) < 1e-4) epGPS = localPosToGPS(ep);

            glm::vec3 awayPt = isStart ? featB.rawPoints[1] : featB.rawPoints[featB.rawPoints.size() - 2];
            glm::vec2 dAway(awayPt.x - ep.x, awayPt.z - ep.z);
            float lenAway = glm::length(dAway);
            if (lenAway > 1e-4f) dAway /= lenAway;

            int cx = getCellCoord(ep.x);
            int cz = getCellCoord(ep.z);

            int bestA = -1;
            int bestSeg = -1;
            float bestDistSq = snapDistance * snapDistance;
            float bestT = 0.0f;
            glm::vec3 bestP(0.0f);
            glm::dvec2 bestGPS = epGPS;

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = grid.find(makeCellKey(cx + dx, cz + dz));
                    if (it == grid.end()) continue;

                    for (const auto& cand : it->second) {
                        int a = cand.featIdx;
                        int s = cand.segIdx;
                        if (a == b) continue;
                        if (!sameGrade(featB, features[a])) continue;

                        const auto& featA = features[a];
                        if (featA.rawPoints.size() < 2 || s + 1 >= (int)featA.rawPoints.size()) continue;

                        const auto& a0 = featA.rawPoints[s];
                        const auto& a1 = featA.rawPoints[s + 1];

                        glm::vec2 segVec(a1.x - a0.x, a1.z - a0.z);
                        float segLenSq = glm::dot(segVec, segVec);
                        if (segLenSq < 1e-6f) continue;
                        float segLen = std::sqrt(segLenSq);
                        glm::vec2 segDir = segVec / segLen;

                        // Project ep onto segment a0 -> a1
                        glm::vec2 epVec(ep.x - a0.x, ep.z - a0.z);
                        float t = glm::dot(epVec, segVec) / segLenSq;
                        float tClamped = std::clamp(t, 0.0f, 1.0f);

                        glm::vec2 proj2D(a0.x + tClamped * segVec.x, a0.z + tClamped * segVec.y);
                        float diffX = ep.x - proj2D.x;
                        float diffZ = ep.z - proj2D.y;
                        float dSq = diffX * diffX + diffZ * diffZ;

                        if (dSq < bestDistSq) {
                            // Check non-parallel angle unless touching within 0.50m
                            float dotD = std::abs(glm::dot(dAway, segDir));
                            if (dSq > 0.50f * 0.50f && dotD > 0.965f) {
                                continue; // Skip near-parallel roads
                            }

                            bestDistSq = dSq;
                            bestA = a;
                            bestSeg = s;
                            bestT = tClamped;

                            float y = glm::mix(a0.y, a1.y, tClamped);
                            bestP = glm::vec3(proj2D.x, y, proj2D.y);

                            if (featA.gpsPoints.size() == featA.rawPoints.size()) {
                                bestGPS = glm::mix(featA.gpsPoints[s], featA.gpsPoints[s + 1], (double)tClamped);
                            } else {
                                bestGPS = localPosToGPS(bestP);
                            }
                            if (std::abs(bestGPS.x) < 1e-4 && std::abs(bestGPS.y) < 1e-4) {
                                bestGPS = localPosToGPS(bestP);
                            }
                        }
                    }
                }
            }

            if (bestA >= 0) {
                const auto& featA = features[bestA];
                int s = bestSeg;
                float segLen = glm::distance(glm::vec2(featA.rawPoints[s].x, featA.rawPoints[s].z),
                                             glm::vec2(featA.rawPoints[s + 1].x, featA.rawPoints[s + 1].z));
                float distFromStart = bestT * segLen;
                float distFromEnd = (1.0f - bestT) * segLen;

                // Total distance along featA from start and end
                float distFromFeatAStart = 0.0f;
                for (int i = 0; i < s; ++i) {
                    distFromFeatAStart += glm::distance(glm::vec2(featA.rawPoints[i].x, featA.rawPoints[i].z),
                                                        glm::vec2(featA.rawPoints[i + 1].x, featA.rawPoints[i + 1].z));
                }
                distFromFeatAStart += bestT * segLen;

                float distFromFeatAEnd = 0.0f;
                for (int i = s + 1; i + 1 < (int)featA.rawPoints.size(); ++i) {
                    distFromFeatAEnd += glm::distance(glm::vec2(featA.rawPoints[i].x, featA.rawPoints[i].z),
                                                      glm::vec2(featA.rawPoints[i + 1].x, featA.rawPoints[i + 1].z));
                }
                distFromFeatAEnd += (1.0f - bestT) * segLen;

                int splitSeg = s;
                float splitT = bestT;
                glm::vec3 splitP = bestP;
                glm::dvec2 splitG = bestGPS;
                bool snapToEndpoint = false;

                // If within minJunctionDistance of featA's endpoints, snap directly to the existing endpoint
                // and avoid creating a short squeezed segment
                if (distFromFeatAStart < minJunctionDistance) {
                    splitSeg = 0;
                    splitT = 0.0f;
                    splitP = featA.rawPoints.front();
                    splitG = (!featA.gpsPoints.empty()) ? featA.gpsPoints.front() : localPosToGPS(splitP);
                    snapToEndpoint = true;
                } else if (distFromFeatAEnd < minJunctionDistance) {
                    splitSeg = (int)featA.rawPoints.size() - 1;
                    splitT = 0.0f;
                    splitP = featA.rawPoints.back();
                    splitG = (!featA.gpsPoints.empty()) ? featA.gpsPoints.back() : localPosToGPS(splitP);
                    snapToEndpoint = true;
                } else if (distFromStart < 1.5f) {
                    splitSeg = s;
                    splitT = 0.0f;
                    splitP = featA.rawPoints[s];
                    splitG = (s < featA.gpsPoints.size()) ? featA.gpsPoints[s] : localPosToGPS(splitP);
                } else if (distFromEnd < 1.5f) {
                    splitSeg = s + 1;
                    splitT = 0.0f;
                    splitP = featA.rawPoints[s + 1];
                    splitG = (s + 1 < featA.gpsPoints.size()) ? featA.gpsPoints[s + 1] : localPosToGPS(splitP);
                }
                if (std::abs(splitG.x) < 1e-4 && std::abs(splitG.y) < 1e-4) splitG = localPosToGPS(splitP);

                // Snap endpoint of feature B to the junction position
                if (isStart) {
                    featB.rawPoints.front() = splitP;
                    if (!featB.gpsPoints.empty()) featB.gpsPoints.front() = splitG;
                } else {
                    featB.rawPoints.back() = splitP;
                    if (!featB.gpsPoints.empty()) featB.gpsPoints.back() = splitG;
                }

                // Only record split on feature A if it's an interior point (not feature A's start or end)
                if (!snapToEndpoint) {
                    bool isFeatAEndpoint = (splitSeg == 0 && splitT <= 1e-4f) ||
                                           (splitSeg >= (int)featA.rawPoints.size() - 1);
                    if (!isFeatAEndpoint) {
                        featSplits[bestA].push_back({ splitSeg, splitT, splitP, splitG });
                        tJunctionsAdded++;
                    }
                }
            }
        }
    }

    // 3. Detect segment-to-segment crossings
    for (int a = 0; a < (int)features.size(); ++a) {
        const auto& featA = features[a];
        if (featA.rawPoints.size() < 2) continue;

        for (int sA = 0; sA + 1 < (int)featA.rawPoints.size(); ++sA) {
            const auto& a0 = featA.rawPoints[sA];
            const auto& a1 = featA.rawPoints[sA + 1];

            glm::vec2 da(a1.x - a0.x, a1.z - a0.z);
            float lenA = glm::length(da);
            if (lenA < 1e-4f) continue;

            int cx = getCellCoord((a0.x + a1.x) * 0.5f);
            int cz = getCellCoord((a0.z + a1.z) * 0.5f);

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = grid.find(makeCellKey(cx + dx, cz + dz));
                    if (it == grid.end()) continue;

                    for (const auto& cand : it->second) {
                        int b = cand.featIdx;
                        int sB = cand.segIdx;

                        if (b < a) continue;
                        if (b == a && sB <= sA + 1) continue;

                        if (!sameGrade(featA, features[b])) continue;

                        const auto& featB = features[b];
                        if (sB + 1 >= (int)featB.rawPoints.size()) continue;

                        const auto& b0 = featB.rawPoints[sB];
                        const auto& b1 = featB.rawPoints[sB + 1];

                        // Bounding box test
                        if (std::max(a0.x, a1.x) < std::min(b0.x, b1.x) - 1e-3f ||
                            std::min(a0.x, a1.x) > std::max(b0.x, b1.x) + 1e-3f ||
                            std::max(a0.z, a1.z) < std::min(b0.z, b1.z) - 1e-3f ||
                            std::min(a0.z, a1.z) > std::max(b0.z, b1.z) + 1e-3f) {
                            continue;
                        }

                        glm::vec2 db(b1.x - b0.x, b1.z - b0.z);
                        float lenB = glm::length(db);
                        if (lenB < 1e-4f) continue;

                        float denom = cross2D(da, db);
                        if (std::abs(denom) < 1e-6f) continue; // Parallel or collinear

                        glm::vec2 dba(b0.x - a0.x, b0.z - a0.z);
                        float tA = cross2D(dba, db) / denom;
                        float tB = cross2D(dba, da) / denom;

                        if (tA > 1e-4f && tA < 1.0f - 1e-4f && tB > 1e-4f && tB < 1.0f - 1e-4f) {
                            glm::vec2 p2D(a0.x + tA * da.x, a0.z + tA * da.y);
                            float y = glm::mix(a0.y, a1.y, tA) * 0.5f + glm::mix(b0.y, b1.y, tB) * 0.5f;
                            glm::vec3 pInt(p2D.x, y, p2D.y);

                            glm::dvec2 gInt = localPosToGPS(pInt);
                            if (featA.gpsPoints.size() == featA.rawPoints.size()) {
                                gInt = glm::mix(featA.gpsPoints[sA], featA.gpsPoints[sA + 1], (double)tA);
                            } else if (featB.gpsPoints.size() == featB.rawPoints.size()) {
                                gInt = glm::mix(featB.gpsPoints[sB], featB.gpsPoints[sB + 1], (double)tB);
                            }
                            if (std::abs(gInt.x) < 1e-4 && std::abs(gInt.y) < 1e-4) {
                                gInt = localPosToGPS(pInt);
                            }

                            // Snap to existing vertices on A if within 1.5m
                            int splitSA = sA;
                            float splitTA = tA;
                            glm::vec3 splitPA = pInt;
                            glm::dvec2 splitGA = gInt;
                            if (tA * lenA < 1.5f) {
                                splitSA = sA; splitTA = 0.0f; splitPA = a0;
                                splitGA = (sA < featA.gpsPoints.size()) ? featA.gpsPoints[sA] : localPosToGPS(splitPA);
                            } else if ((1.0f - tA) * lenA < 1.5f) {
                                splitSA = sA + 1; splitTA = 0.0f; splitPA = a1;
                                splitGA = (sA + 1 < featA.gpsPoints.size()) ? featA.gpsPoints[sA + 1] : localPosToGPS(splitPA);
                            }
                            if (std::abs(splitGA.x) < 1e-4 && std::abs(splitGA.y) < 1e-4) splitGA = localPosToGPS(splitPA);

                            // Snap to existing vertices on B if within 1.5m
                            int splitSB = sB;
                            float splitTB = tB;
                            glm::vec3 splitPB = pInt;
                            glm::dvec2 splitGB = gInt;
                            if (tB * lenB < 1.5f) {
                                splitSB = sB; splitTB = 0.0f; splitPB = b0;
                                splitGB = (sB < featB.gpsPoints.size()) ? featB.gpsPoints[sB] : localPosToGPS(splitPB);
                            } else if ((1.0f - tB) * lenB < 1.5f) {
                                splitSB = sB + 1; splitTB = 0.0f; splitPB = b1;
                                splitGB = (sB + 1 < featB.gpsPoints.size()) ? featB.gpsPoints[sB + 1] : localPosToGPS(splitPB);
                            }
                            if (std::abs(splitGB.x) < 1e-4 && std::abs(splitGB.y) < 1e-4) splitGB = localPosToGPS(splitPB);

                            if (splitTA == 0.0f && splitTB != 0.0f) {
                                splitPB = splitPA; splitGB = splitGA;
                            } else if (splitTB == 0.0f && splitTA != 0.0f) {
                                splitPA = splitPB; splitGA = splitGB;
                            }

                            bool isEndA = (splitSA == 0 && splitTA <= 1e-4f) || (splitSA >= (int)featA.rawPoints.size() - 1);
                            if (!isEndA) {
                                featSplits[a].push_back({ splitSA, splitTA, splitPA, splitGA });
                            }

                            bool isEndB = (splitSB == 0 && splitTB <= 1e-4f) || (splitSB >= (int)featB.rawPoints.size() - 1);
                            if (!isEndB) {
                                featSplits[b].push_back({ splitSB, splitTB, splitPB, splitGB });
                            }

                            crossingsAdded++;
                        }
                    }
                }
            }
        }
    }

    // 4. Sort, deduplicate, and slice features at all recorded split locations
    std::vector<RawRoadFeature> newFeatures;
    newFeatures.reserve(features.size() + featSplits.size() * 2);

    int totalSplitsApplied = 0;
    uint32_t nextId = 1;
    for (const auto& f : features) nextId = std::max(nextId, f.id + 1);

    for (int f = 0; f < (int)features.size(); ++f) {
        auto& feat = features[f];
        auto& splits = featSplits[f];

        if (splits.empty() || feat.rawPoints.size() < 2) {
            newFeatures.push_back(std::move(feat));
            continue;
        }

        std::sort(splits.begin(), splits.end(), [](const SplitPoint& a, const SplitPoint& b) {
            return a.key() < b.key();
        });

        std::vector<SplitPoint> uniqueSplits;
        for (const auto& sp : splits) {
            if (sp.segIdx == 0 && sp.t <= 1e-4f) continue;
            if (sp.segIdx >= (int)feat.rawPoints.size() - 1) continue;
            if (sp.segIdx == (int)feat.rawPoints.size() - 2 && sp.t >= 1.0f - 1e-4f) continue;

            // Check distance along feat from start and end
            float spDistFromStart = 0.0f;
            for (int i = 0; i < sp.segIdx; ++i) {
                spDistFromStart += glm::distance(glm::vec2(feat.rawPoints[i].x, feat.rawPoints[i].z),
                                                 glm::vec2(feat.rawPoints[i + 1].x, feat.rawPoints[i + 1].z));
            }
            float sLen = glm::distance(glm::vec2(feat.rawPoints[sp.segIdx].x, feat.rawPoints[sp.segIdx].z),
                                       glm::vec2(feat.rawPoints[sp.segIdx + 1].x, feat.rawPoints[sp.segIdx + 1].z));
            spDistFromStart += sp.t * sLen;

            float featTotalLen = 0.0f;
            for (size_t i = 0; i + 1 < feat.rawPoints.size(); ++i) {
                featTotalLen += glm::distance(glm::vec2(feat.rawPoints[i].x, feat.rawPoints[i].z),
                                              glm::vec2(feat.rawPoints[i + 1].x, feat.rawPoints[i + 1].z));
            }
            float spDistFromEnd = featTotalLen - spDistFromStart;

            if (spDistFromStart < minJunctionDistance) {
                // Too close to feat start: snap any intersecting road endpoint that was placed at sp.pos to feat.rawPoints.front()
                glm::vec3 frontP = feat.rawPoints.front();
                glm::dvec2 frontG = (!feat.gpsPoints.empty()) ? feat.gpsPoints.front() : localPosToGPS(frontP);
                for (auto& other : features) {
                    if (other.rawPoints.empty()) continue;
                    if (glm::distance(glm::vec2(other.rawPoints.front().x, other.rawPoints.front().z),
                                      glm::vec2(sp.pos.x, sp.pos.z)) < 2.0f) {
                        other.rawPoints.front() = frontP;
                        if (!other.gpsPoints.empty()) other.gpsPoints.front() = frontG;
                    }
                    if (glm::distance(glm::vec2(other.rawPoints.back().x, other.rawPoints.back().z),
                                      glm::vec2(sp.pos.x, sp.pos.z)) < 2.0f) {
                        other.rawPoints.back() = frontP;
                        if (!other.gpsPoints.empty()) other.gpsPoints.back() = frontG;
                    }
                }
                continue;
            }
            if (spDistFromEnd < minJunctionDistance) {
                // Too close to feat end: snap to feat.rawPoints.back()
                glm::vec3 backP = feat.rawPoints.back();
                glm::dvec2 backG = (!feat.gpsPoints.empty()) ? feat.gpsPoints.back() : localPosToGPS(backP);
                for (auto& other : features) {
                    if (other.rawPoints.empty()) continue;
                    if (glm::distance(glm::vec2(other.rawPoints.front().x, other.rawPoints.front().z),
                                      glm::vec2(sp.pos.x, sp.pos.z)) < 2.0f) {
                        other.rawPoints.front() = backP;
                        if (!other.gpsPoints.empty()) other.gpsPoints.front() = backG;
                    }
                    if (glm::distance(glm::vec2(other.rawPoints.back().x, other.rawPoints.back().z),
                                      glm::vec2(sp.pos.x, sp.pos.z)) < 2.0f) {
                        other.rawPoints.back() = backP;
                        if (!other.gpsPoints.empty()) other.gpsPoints.back() = backG;
                    }
                }
                continue;
            }

            // Check if this split is too close to an existing unique split on this feature
            bool mergedWithPrev = false;
            for (auto& u : uniqueSplits) {
                float dBetween = glm::distance(glm::vec2(sp.pos.x, sp.pos.z), glm::vec2(u.pos.x, u.pos.z));
                if (dBetween < minJunctionDistance) {
                    // Merge into single junction at midpoint to prevent tiny squeezed road
                    glm::vec3 oldUPos = u.pos;
                    glm::vec3 oldSpPos = sp.pos;
                    glm::vec3 mergedP = (oldUPos + oldSpPos) * 0.5f;
                    glm::dvec2 mergedG = (u.gps + sp.gps) * 0.5;

                    u.pos = mergedP;
                    u.gps = mergedG;

                    // Update any features whose endpoints were snapped to oldUPos or oldSpPos
                    for (auto& other : features) {
                        if (other.rawPoints.empty()) continue;
                        if (glm::distance(glm::vec2(other.rawPoints.front().x, other.rawPoints.front().z),
                                          glm::vec2(oldUPos.x, oldUPos.z)) < 2.0f ||
                            glm::distance(glm::vec2(other.rawPoints.front().x, other.rawPoints.front().z),
                                          glm::vec2(oldSpPos.x, oldSpPos.z)) < 2.0f) {
                            other.rawPoints.front() = mergedP;
                            if (!other.gpsPoints.empty()) other.gpsPoints.front() = mergedG;
                        }
                        if (glm::distance(glm::vec2(other.rawPoints.back().x, other.rawPoints.back().z),
                                          glm::vec2(oldUPos.x, oldUPos.z)) < 2.0f ||
                            glm::distance(glm::vec2(other.rawPoints.back().x, other.rawPoints.back().z),
                                          glm::vec2(oldSpPos.x, oldSpPos.z)) < 2.0f) {
                            other.rawPoints.back() = mergedP;
                            if (!other.gpsPoints.empty()) other.gpsPoints.back() = mergedG;
                        }
                    }
                    mergedWithPrev = true;
                    break;
                }
            }
            if (!mergedWithPrev) {
                uniqueSplits.push_back(sp);
            }
        }

        if (uniqueSplits.empty()) {
            newFeatures.push_back(std::move(feat));
            continue;
        }

        totalSplitsApplied += (int)uniqueSplits.size();

        RawRoadFeature curSub = feat;
        curSub.rawPoints.clear();
        curSub.gpsPoints.clear();
        curSub.rawPoints.push_back(feat.rawPoints.front());
        if (!feat.gpsPoints.empty()) curSub.gpsPoints.push_back(feat.gpsPoints.front());

        size_t splitIdx = 0;
        int subPart = 1;

        for (int s = 0; s + 1 < (int)feat.rawPoints.size(); ++s) {
            while (splitIdx < uniqueSplits.size() && uniqueSplits[splitIdx].segIdx == s) {
                const auto& sp = uniqueSplits[splitIdx++];

                if (curSub.rawPoints.empty() ||
                    glm::distance(glm::vec2(curSub.rawPoints.back().x, curSub.rawPoints.back().z),
                                  glm::vec2(sp.pos.x, sp.pos.z)) > 0.05f) {
                    curSub.rawPoints.push_back(sp.pos);
                    if (!feat.gpsPoints.empty()) curSub.gpsPoints.push_back(sp.gps);
                }

                if (curSub.rawPoints.size() >= 2) {
                    curSub.id = nextId++;
                    curSub.osmId = feat.osmId + "_p" + std::to_string(subPart++);
                    newFeatures.push_back(curSub);
                }

                curSub = feat;
                curSub.rawPoints.clear();
                curSub.gpsPoints.clear();
                curSub.rawPoints.push_back(sp.pos);
                if (!feat.gpsPoints.empty()) curSub.gpsPoints.push_back(sp.gps);
            }

            const auto& ptNext = feat.rawPoints[s + 1];
            if (curSub.rawPoints.empty() ||
                glm::distance(glm::vec2(curSub.rawPoints.back().x, curSub.rawPoints.back().z),
                              glm::vec2(ptNext.x, ptNext.z)) > 0.05f) {
                curSub.rawPoints.push_back(ptNext);
                if (!feat.gpsPoints.empty() && s + 1 < (int)feat.gpsPoints.size()) {
                    curSub.gpsPoints.push_back(feat.gpsPoints[s + 1]);
                }
            }
        }

        if (curSub.rawPoints.size() >= 2) {
            curSub.id = nextId++;
            curSub.osmId = feat.osmId + "_p" + std::to_string(subPart++);
            newFeatures.push_back(curSub);
        }
    }

    features = std::move(newFeatures);
    for (auto& f : features) {
        ensureValidGPS(f);
    }

    std::cout << "RoadGenerator: Detected & split " << totalSplitsApplied
              << " intersections (" << tJunctionsAdded << " T-junctions, "
              << crossingsAdded << " crossings) into "
              << features.size() << " connected road features." << std::endl;
}

void RoadGenerator::absorbMicroSegments(std::vector<RawRoadFeature>& features, float minSegmentLength) {
    if (features.empty()) return;

    auto getFeatureLength = [](const RawRoadFeature& f) -> float {
        float l = 0.0f;
        for (size_t k = 0; k + 1 < f.rawPoints.size(); ++k) {
            l += glm::distance(glm::vec2(f.rawPoints[k].x, f.rawPoints[k].z),
                               glm::vec2(f.rawPoints[k + 1].x, f.rawPoints[k + 1].z));
        }
        return l;
    };

    struct NodeConn {
        int featIdx;
        bool isStart;
    };

    auto coordsKey = [](const glm::vec3& p) -> int64_t {
        // Quantize to 0.5m grid
        int64_t qx = (int64_t)std::floor(p.x * 2.0f + 0.5f);
        int64_t qz = (int64_t)std::floor(p.z * 2.0f + 0.5f);
        return (qx << 32) ^ (qz & 0xFFFFFFFFLL);
    };

    int absorbedCount = 0;
    bool changed = true;
    int pass = 0;
    while (changed && pass < 10) {
        changed = false;
        pass++;

        // Build endpoint connectivity map
        std::unordered_map<int64_t, std::vector<NodeConn>> nodeMap;
        nodeMap.reserve(features.size() * 2);
        for (size_t i = 0; i < features.size(); ++i) {
            if (features[i].rawPoints.size() < 2) continue;
            nodeMap[coordsKey(features[i].rawPoints.front())].push_back({ (int)i, true });
            nodeMap[coordsKey(features[i].rawPoints.back())].push_back({ (int)i, false });
        }

        std::vector<bool> removed(features.size(), false);

        for (size_t i = 0; i < features.size(); ++i) {
            if (removed[i]) continue;
            auto& f = features[i];
            if (f.rawPoints.size() < 2) continue;

            float L = getFeatureLength(f);
            if (L >= minSegmentLength) continue;

            auto k0 = coordsKey(f.rawPoints.front());
            auto k1 = coordsKey(f.rawPoints.back());

            auto it0 = nodeMap.find(k0);
            auto it1 = nodeMap.find(k1);
            if (it0 == nodeMap.end() || it1 == nodeMap.end()) continue;

            const auto& c0 = it0->second;
            const auto& c1 = it1->second;

            // Case D: BOTH ends are multi-way junctions (>= 3 conns each).
            // This is a very short road squeezed between two junctions.
            // Automatically delete the squeezed road and merge all connected roads at the midpoint!
            if (c0.size() >= 3 && c1.size() >= 3) {
                glm::vec3 p0 = f.rawPoints.front();
                glm::vec3 p1 = f.rawPoints.back();
                glm::vec3 mergedCenter = (p0 + p1) * 0.5f;

                if (f.isBridge || f.layer > 0 || f.elevation > 1.0f) {
                    float elev = std::max(f.elevation, (f.layer > 0 ? (float)f.layer * 5.5f : 5.5f));
                    mergedCenter.y = elev;
                }

                glm::dvec2 g0 = (!f.gpsPoints.empty()) ? f.gpsPoints.front() : localPosToGPS(p0);
                glm::dvec2 g1 = (!f.gpsPoints.empty()) ? f.gpsPoints.back() : localPosToGPS(p1);
                glm::dvec2 mergedGPS = (g0 + g1) * 0.5;

                // Move all other features meeting at k0 to mergedCenter
                for (const auto& conn : c0) {
                    if (conn.featIdx == (int)i || removed[conn.featIdx]) continue;
                    auto& otherF = features[conn.featIdx];
                    if (otherF.rawPoints.empty()) continue;

                    if (conn.isStart) {
                        otherF.rawPoints.front() = mergedCenter;
                        if (!otherF.gpsPoints.empty()) otherF.gpsPoints.front() = mergedGPS;
                        while (otherF.rawPoints.size() > 2 &&
                               glm::distance(glm::vec2(otherF.rawPoints[1].x, otherF.rawPoints[1].z),
                                             glm::vec2(mergedCenter.x, mergedCenter.z)) < 2.5f) {
                            otherF.rawPoints.erase(otherF.rawPoints.begin() + 1);
                            if (otherF.gpsPoints.size() > otherF.rawPoints.size()) {
                                otherF.gpsPoints.erase(otherF.gpsPoints.begin() + 1);
                            }
                        }
                    } else {
                        otherF.rawPoints.back() = mergedCenter;
                        if (!otherF.gpsPoints.empty()) otherF.gpsPoints.back() = mergedGPS;
                        while (otherF.rawPoints.size() > 2 &&
                               glm::distance(glm::vec2(otherF.rawPoints[otherF.rawPoints.size() - 2].x,
                                                       otherF.rawPoints[otherF.rawPoints.size() - 2].z),
                                             glm::vec2(mergedCenter.x, mergedCenter.z)) < 2.5f) {
                            otherF.rawPoints.erase(otherF.rawPoints.end() - 2);
                            if (otherF.gpsPoints.size() > otherF.rawPoints.size()) {
                                otherF.gpsPoints.erase(otherF.gpsPoints.end() - 2);
                            }
                        }
                    }
                }

                // Move all other features meeting at k1 to mergedCenter
                for (const auto& conn : c1) {
                    if (conn.featIdx == (int)i || removed[conn.featIdx]) continue;
                    auto& otherF = features[conn.featIdx];
                    if (otherF.rawPoints.empty()) continue;

                    if (conn.isStart) {
                        otherF.rawPoints.front() = mergedCenter;
                        if (!otherF.gpsPoints.empty()) otherF.gpsPoints.front() = mergedGPS;
                        while (otherF.rawPoints.size() > 2 &&
                               glm::distance(glm::vec2(otherF.rawPoints[1].x, otherF.rawPoints[1].z),
                                             glm::vec2(mergedCenter.x, mergedCenter.z)) < 2.5f) {
                            otherF.rawPoints.erase(otherF.rawPoints.begin() + 1);
                            if (otherF.gpsPoints.size() > otherF.rawPoints.size()) {
                                otherF.gpsPoints.erase(otherF.gpsPoints.begin() + 1);
                            }
                        }
                    } else {
                        otherF.rawPoints.back() = mergedCenter;
                        if (!otherF.gpsPoints.empty()) otherF.gpsPoints.back() = mergedGPS;
                        while (otherF.rawPoints.size() > 2 &&
                               glm::distance(glm::vec2(otherF.rawPoints[otherF.rawPoints.size() - 2].x,
                                                       otherF.rawPoints[otherF.rawPoints.size() - 2].z),
                                             glm::vec2(mergedCenter.x, mergedCenter.z)) < 2.5f) {
                            otherF.rawPoints.erase(otherF.rawPoints.end() - 2);
                            if (otherF.gpsPoints.size() > otherF.rawPoints.size()) {
                                otherF.gpsPoints.erase(otherF.gpsPoints.end() - 2);
                            }
                        }
                    }
                }

                removed[i] = true;
                absorbedCount++;
                changed = true;
            }
            // Case A: k0 is multi-way (>= 3 conns) and k1 is 2-way (== 2 conns)
            else if (c0.size() >= 3 && c1.size() == 2) {
                int otherIdx = -1;
                bool otherIsStart = false;
                for (const auto& conn : c1) {
                    if (conn.featIdx != (int)i && !removed[conn.featIdx]) {
                        otherIdx = conn.featIdx;
                        otherIsStart = conn.isStart;
                        break;
                    }
                }
                if (otherIdx >= 0 && otherIdx < (int)features.size() && !removed[otherIdx]) {
                    auto& otherF = features[otherIdx];
                    if (f.isBridge || f.layer > 0 || f.elevation > 1.0f) {
                        float elev = std::max(f.elevation, (f.layer > 0 ? (float)f.layer * 5.5f : 5.5f));
                        f.rawPoints.front().y = elev;
                        f.rawPoints.back().y = elev;
                    }
                    if (otherIsStart) {
                        otherF.rawPoints.front() = f.rawPoints.front();
                    } else {
                        otherF.rawPoints.back() = f.rawPoints.front();
                    }
                    removed[i] = true;
                    absorbedCount++;
                    changed = true;
                }
            }
            // Case B: k1 is multi-way (>= 3 conns) and k0 is 2-way (== 2 conns)
            else if (c1.size() >= 3 && c0.size() == 2) {
                int otherIdx = -1;
                bool otherIsStart = false;
                for (const auto& conn : c0) {
                    if (conn.featIdx != (int)i && !removed[conn.featIdx]) {
                        otherIdx = conn.featIdx;
                        otherIsStart = conn.isStart;
                        break;
                    }
                }
                if (otherIdx >= 0 && otherIdx < (int)features.size() && !removed[otherIdx]) {
                    auto& otherF = features[otherIdx];
                    if (f.isBridge || f.layer > 0 || f.elevation > 1.0f) {
                        float elev = std::max(f.elevation, (f.layer > 0 ? (float)f.layer * 5.5f : 5.5f));
                        f.rawPoints.front().y = elev;
                        f.rawPoints.back().y = elev;
                    }
                    if (otherIsStart) {
                        otherF.rawPoints.front() = f.rawPoints.back();
                    } else {
                        otherF.rawPoints.back() = f.rawPoints.back();
                    }
                    removed[i] = true;
                    absorbedCount++;
                    changed = true;
                }
            }
            // Case C: both ends are 2-way (== 2 conns) -> merge into c0's other feature
            else if (c0.size() == 2 && c1.size() == 2) {
                int otherIdx = -1;
                bool otherIsStart = false;
                for (const auto& conn : c0) {
                    if (conn.featIdx != (int)i && !removed[conn.featIdx]) {
                        otherIdx = conn.featIdx;
                        otherIsStart = conn.isStart;
                        break;
                    }
                }
                if (otherIdx >= 0 && otherIdx < (int)features.size() && !removed[otherIdx]) {
                    auto& otherF = features[otherIdx];
                    if (f.isBridge || f.layer > 0 || f.elevation > 1.0f) {
                        float elev = std::max(f.elevation, (f.layer > 0 ? (float)f.layer * 5.5f : 5.5f));
                        f.rawPoints.front().y = elev;
                        f.rawPoints.back().y = elev;
                    }
                    if (otherIsStart) {
                        otherF.rawPoints.front() = f.rawPoints.back();
                    } else {
                        otherF.rawPoints.back() = f.rawPoints.back();
                    }
                    removed[i] = true;
                    absorbedCount++;
                    changed = true;
                }
            }
        }

        if (changed) {
            std::vector<RawRoadFeature> compacted;
            compacted.reserve(features.size());
            for (size_t k = 0; k < features.size(); ++k) {
                if (!removed[k]) compacted.push_back(std::move(features[k]));
            }
            features = std::move(compacted);
        }
    }

    if (absorbedCount > 0) {
        std::cout << "RoadGenerator: Absorbed " << absorbedCount
                  << " unphysical micro-segments (< " << minSegmentLength << "m)" << std::endl;
    }
    for (auto& feat : features) {
        ensureValidGPS(feat);
    }
}

void RoadGenerator::sanitizeRoadFeatures(std::vector<RawRoadFeature>& features,
                                         float minControlPointDist,
                                         float minSegmentLength) {
    if (features.empty()) return;

    // ------------------------------------------------------------------------
    // Stage 1: Control Point Subsampling & Jitter Suppression
    // ------------------------------------------------------------------------
    int cpPruned = 0;
    for (auto& feat : features) {
        if (feat.rawPoints.size() < 2) continue;

        std::vector<glm::vec3> simplified;
        simplified.reserve(feat.rawPoints.size());
        simplified.push_back(feat.rawPoints.front());

        for (size_t i = 1; i + 1 < feat.rawPoints.size(); ++i) {
            float dist = glm::distance(glm::vec2(feat.rawPoints[i].x, feat.rawPoints[i].z),
                                       glm::vec2(simplified.back().x, simplified.back().z));
            if (dist >= minControlPointDist) {
                simplified.push_back(feat.rawPoints[i]);
            } else {
                cpPruned++;
            }
        }
        simplified.push_back(feat.rawPoints.back());
        feat.rawPoints = std::move(simplified);

        if (feat.rawPoints.size() > 2) {
            std::vector<glm::vec3> filtered;
            filtered.reserve(feat.rawPoints.size());
            filtered.push_back(feat.rawPoints.front());
            for (size_t i = 1; i + 1 < feat.rawPoints.size(); ++i) {
                float dPrev = glm::distance(glm::vec2(feat.rawPoints[i].x, feat.rawPoints[i].z),
                                            glm::vec2(filtered.back().x, filtered.back().z));
                float dNext = glm::distance(glm::vec2(feat.rawPoints[i].x, feat.rawPoints[i].z),
                                            glm::vec2(feat.rawPoints.back().x, feat.rawPoints.back().z));
                if (dPrev >= 3.50f && dNext >= 3.50f) {
                    filtered.push_back(feat.rawPoints[i]);
                } else {
                    cpPruned++;
                }
            }
            filtered.push_back(feat.rawPoints.back());
            feat.rawPoints = std::move(filtered);
        }

        ensureValidGPS(feat);
    }

    if (cpPruned > 0) {
        std::cout << "RoadGenerator: Pruned " << cpPruned
                  << " noisy/too-close control points across road network" << std::endl;
    }

    // ------------------------------------------------------------------------
    // Stage 2: Iterative Micro-Segment Absorption (< minSegmentLength)
    // ------------------------------------------------------------------------
    absorbMicroSegments(features, minSegmentLength);

    // ------------------------------------------------------------------------
    // Stage 3: Continuous Corridor Stitching
    // ------------------------------------------------------------------------
    stitchCollinearFeatures(features);
    for (auto& feat : features) {
        ensureValidGPS(feat);
    }

    // ------------------------------------------------------------------------
    // Stage 4: Detect Intersections & Split Corridors to Create Junctions
    // ------------------------------------------------------------------------
    splitIntersectingFeatures(features, 6.0f, minSegmentLength);
    for (auto& feat : features) {
        ensureValidGPS(feat);
    }

    // ------------------------------------------------------------------------
    // Stage 5: Absorb any micro-segments created by splitting
    // ------------------------------------------------------------------------
    absorbMicroSegments(features, minSegmentLength);
}

void RoadGenerator::processFeatures(const std::vector<RawRoadFeature>& rawFeatures,
                                    float laneWidth,
                                    float shoulderWidth,
                                    float sampleStep,
                                    std::vector<ProcessedRoadSegment>& outSegments,
                                    bool useOverrides,
                                    bool generateVisualMeshes) {
    outSegments.clear();

    std::vector<RawRoadFeature> features = rawFeatures;
    sanitizeRoadFeatures(features, 6.0f, 26.0f);

    // Propagate bridge elevations to approach ramps
    // Ground roads (layer == 0 && !isBridge) connecting to an elevated road/bridge (layer > 0 || isBridge)
    // smoothly transition their vertical profile from the bridge height down to ground level (0.05m).
    // Fast spatial map of elevated road endpoints
    auto getCellKey = [](float x, float z) -> int64_t {
        int64_t gx = static_cast<int64_t>(std::floor(x));
        int64_t gz = static_cast<int64_t>(std::floor(z));
        return (gx << 32) ^ (gz & 0xFFFFFFFFLL);
    };

    std::unordered_map<int64_t, std::vector<glm::vec3>> elevatedMap;
    for (const auto& other : features) {
        if (other.isBridge || other.layer > 0 || other.rawPoints.front().y > 1.0f || other.rawPoints.back().y > 1.0f) {
            if (!other.rawPoints.empty()) {
                elevatedMap[getCellKey(other.rawPoints.front().x, other.rawPoints.front().z)].push_back(other.rawPoints.front());
                elevatedMap[getCellKey(other.rawPoints.back().x, other.rawPoints.back().z)].push_back(other.rawPoints.back());
            }
        }
    }

    auto findElevatedElev = [&](const glm::vec3& pt) -> float {
        int64_t bgx = static_cast<int64_t>(std::floor(pt.x));
        int64_t bgz = static_cast<int64_t>(std::floor(pt.z));
        float maxElev = pt.y;
        for (int64_t dx = -1; dx <= 1; ++dx) {
            for (int64_t dz = -1; dz <= 1; ++dz) {
                int64_t k = ((bgx + dx) << 32) ^ ((bgz + dz) & 0xFFFFFFFFLL);
                auto it = elevatedMap.find(k);
                if (it != elevatedMap.end()) {
                    for (const auto& ep : it->second) {
                        if (glm::distance(glm::vec2(pt.x, pt.z), glm::vec2(ep.x, ep.z)) < 0.6f) {
                            maxElev = std::max(maxElev, ep.y);
                        }
                    }
                }
            }
        }
        return maxElev;
    };

    for (size_t i = 0; i < features.size(); ++i) {
        auto& f = features[i];
        if (f.isBridge || f.layer > 0 || f.rawPoints.size() < 2) continue;

        float startElev = findElevatedElev(f.rawPoints.front());
        float endElev = findElevatedElev(f.rawPoints.back());

        if (std::abs(startElev - endElev) > 0.5f) {
            float totalLen = 0.0f;
            for (size_t k = 0; k + 1 < f.rawPoints.size(); ++k) {
                totalLen += glm::distance(glm::vec2(f.rawPoints[k].x, f.rawPoints[k].z),
                                          glm::vec2(f.rawPoints[k + 1].x, f.rawPoints[k + 1].z));
            }
            float accDist = 0.0f;
            f.rawPoints.front().y = startElev;
            for (size_t k = 1; k < f.rawPoints.size(); ++k) {
                accDist += glm::distance(glm::vec2(f.rawPoints[k].x, f.rawPoints[k].z),
                                         glm::vec2(f.rawPoints[k - 1].x, f.rawPoints[k - 1].z));
                float t = (totalLen > 1e-4f) ? std::clamp(accDist / totalLen, 0.0f, 1.0f) : 1.0f;
                float s = 3.0f * t * t - 2.0f * t * t * t; // Smoothstep vertical curve
                f.rawPoints[k].y = glm::mix(startElev, endElev, s);
            }
            f.rawPoints.back().y = endElev;
            f.elevation = (startElev + endElev) * 0.5f;
        } else if (startElev > 1.0f && endElev > 1.0f) {
            for (auto& pt : f.rawPoints) pt.y = startElev;
            f.elevation = startElev;
        }
    }

    outSegments.reserve(features.size());

    for (const auto& feat : features) {
        ProcessedRoadSegment seg;
        seg.id = feat.id;
        seg.osmId = feat.osmId;
        seg.name = feat.name.empty() ? feat.highway : feat.name;
        seg.highway = feat.highway;
        seg.ref = feat.ref;
        seg.layer = feat.layer;
        seg.isBridge = feat.isBridge;
        seg.isTunnel = feat.isTunnel;
        seg.elevation = feat.elevation;

        // Subdivide long spans (> 12m) once on initial ingestion so curves have sufficient DOFs to flex
        std::vector<glm::vec3> initialPts;
        std::vector<glm::dvec2> initialGps;
        if (!feat.rawPoints.empty()) {
            initialPts.reserve(feat.rawPoints.size() * 2);
            initialPts.push_back(feat.rawPoints.front());
            if (!feat.gpsPoints.empty()) {
                initialGps.reserve(feat.gpsPoints.size() * 2);
                initialGps.push_back(feat.gpsPoints.front());
            }
            for (size_t k = 0; k + 1 < feat.rawPoints.size(); ++k) {
                const auto& p0 = feat.rawPoints[k];
                const auto& p1 = feat.rawPoints[k + 1];
                float dist2D = glm::distance(glm::vec2(p0.x, p0.z), glm::vec2(p1.x, p1.z));
                int steps = std::max(1, (int)std::ceil(dist2D / 12.0f));
                for (int s = 1; s <= steps; ++s) {
                    float f = (float)s / (float)steps;
                    initialPts.push_back(glm::mix(p0, p1, f));
                    if (!feat.gpsPoints.empty() && k + 1 < feat.gpsPoints.size()) {
                        initialGps.push_back(glm::mix(feat.gpsPoints[k], feat.gpsPoints[k + 1], (double)f));
                    }
                }
            }
        }
        seg.controlPoints = std::move(initialPts);
        seg.gpsPoints = std::move(initialGps);
        seg.sampleStep = sampleStep;

        if (useOverrides) {
            seg.lanes = feat.lanes;
            seg.oneway = feat.oneway;
            seg.laneWidth = laneWidth;
            seg.shoulderWidth = shoulderWidth;
            seg.widthSource = "Custom Global Tweak";
        } else {
            deduceRoadParameters(feat, seg.laneWidth, seg.shoulderWidth, seg.lanes, seg.oneway, seg.widthSource);
        }

        seg.totalWidth = (float)seg.lanes * seg.laneWidth + 2.0f * seg.shoulderWidth;

        seg.splinePoints = generateSpline(seg.controlPoints, sampleStep);
        if (seg.splinePoints.size() >= 2) {
            seg.length = seg.splinePoints.back().distance;

            // Check if segment intersects central 500m x 500m square (X in [-250, 250], Z in [-250, 250])
            bool in500 = false;
            for (const auto& sp : seg.splinePoints) {
                if (std::abs(sp.pos.x) <= 250.0f && std::abs(sp.pos.z) <= 250.0f) {
                    in500 = true;
                    break;
                }
            }
            seg.in500mChunk = in500;

            if (generateVisualMeshes) {
                std::vector<Vertex> verts;
                std::vector<uint32_t> idxs;
                generateRibbonGeometry(seg.splinePoints, seg.totalWidth, seg.lanes, seg.oneway, seg.shoulderWidth, verts, idxs);

                if (!verts.empty() && !idxs.empty()) {
                    seg.visualMesh = std::make_shared<Mesh>(verts.data(), (Uint32)verts.size(),
                                                            idxs.data(), (Uint32)idxs.size());
                }
            }
            outSegments.push_back(seg);
        }
    }

    std::cout << "RoadGenerator: Processed " << outSegments.size()
              << " road segments (" << std::count_if(outSegments.begin(), outSegments.end(),
                   [](const ProcessedRoadSegment& s){ return s.in500mChunk; })
              << " inside 500m chunk)" << std::endl;
}

std::vector<RoadSplinePoint> RoadGenerator::trimSpline(const std::vector<RoadSplinePoint>& spline,
                                                      float startSetback,
                                                      float endSetback) {
    if (spline.size() < 2) return spline;
    float totalLen = spline.back().distance;
    if (startSetback <= 0.001f && endSetback <= 0.001f) {
        return spline;
    }

    float sMin = std::clamp(startSetback, 0.0f, totalLen * 0.45f);
    float sMax = std::clamp(totalLen - endSetback, sMin + 0.5f, totalLen);

    auto sampleAt = [&](float targetDist) -> RoadSplinePoint {
        if (targetDist <= spline.front().distance) return spline.front();
        if (targetDist >= spline.back().distance) return spline.back();
        for (size_t i = 0; i + 1 < spline.size(); ++i) {
            if (targetDist >= spline[i].distance && targetDist <= spline[i + 1].distance) {
                float segLen = spline[i + 1].distance - spline[i].distance;
                float t = (segLen > 1e-4f) ? (targetDist - spline[i].distance) / segLen : 0.0f;
                RoadSplinePoint sp;
                sp.pos = glm::mix(spline[i].pos, spline[i + 1].pos, t);
                sp.tangent = glm::normalize(glm::mix(spline[i].tangent, spline[i + 1].tangent, t));
                sp.normal = glm::normalize(glm::mix(spline[i].normal, spline[i + 1].normal, t));
                sp.distance = targetDist;
                return sp;
            }
        }
        return spline.back();
    };

    std::vector<RoadSplinePoint> trimmed;
    trimmed.reserve(spline.size());

    // Start cap
    trimmed.push_back(sampleAt(sMin));

    // Intermediate points strictly between sMin and sMax
    for (const auto& sp : spline) {
        if (sp.distance > sMin + 0.15f && sp.distance < sMax - 0.15f) {
            trimmed.push_back(sp);
        }
    }

    // End cap
    trimmed.push_back(sampleAt(sMax));
    return trimmed;
}

static std::vector<glm::vec3> generateFilletArc(const glm::vec3& P0, const glm::vec3& P1,
                                                const glm::vec2& dir0, const glm::vec2& dir1,
                                                int steps = 6) {
    std::vector<glm::vec3> arc;
    arc.reserve(steps + 1);

    glm::vec2 p0(P0.x, P0.z);
    glm::vec2 p1(P1.x, P1.z);
    glm::vec2 chord = p1 - p0;
    float chordLen = glm::length(chord);
    if (chordLen < 1e-4f) {
        arc.push_back(P0);
        arc.push_back(P1);
        return arc;
    }

    // Solve for intersection of ray p0 + s * dir0 and p1 - t * dir1
    float det = dir0.x * dir1.y - dir0.y * dir1.x;
    glm::vec2 Q = (p0 + p1) * 0.5f;

    if (std::abs(det) > 1e-4f) {
        float s = (chord.x * dir1.y - chord.y * dir1.x) / det;
        float t = (dir0.x * chord.y - dir0.y * chord.x) / det;
        if (s > 0.0f && t > 0.0f) {
            glm::vec2 inter = p0 + dir0 * s;
            float maxOffset = chordLen * 1.25f;
            if (glm::distance(inter, p0) <= maxOffset && glm::distance(inter, p1) <= maxOffset) {
                Q = inter;
            }
        }
    }

    for (int i = 0; i <= steps; ++i) {
        float u = (float)i / (float)steps;
        float invU = 1.0f - u;
        glm::vec2 pt2D = invU * invU * p0 + 2.0f * invU * u * Q + u * u * p1;
        float elev = glm::mix(P0.y, P1.y, u);
        arc.push_back(glm::vec3(pt2D.x, elev, pt2D.y));
    }
    return arc;
}

static void generateCurbReturnRibbon(const std::vector<glm::vec3>& arc,
                                     float shoulderWidth,
                                     std::vector<Vertex>& outVerts,
                                     std::vector<uint32_t>& outIndices) {
    if (arc.size() < 2) return;
    uint32_t base = (uint32_t)outVerts.size();
    float accDist = 0.0f;
    float wS = std::clamp(shoulderWidth, 1.2f, 2.5f);

    for (size_t i = 0; i < arc.size(); ++i) {
        glm::vec3 pt = arc[i];
        if (i > 0) accDist += glm::distance(pt, arc[i - 1]);

        glm::vec3 T(0.0f);
        if (i == 0) T = glm::normalize(arc[1] - arc[0]);
        else if (i + 1 == arc.size()) T = glm::normalize(arc.back() - arc[arc.size() - 2]);
        else T = glm::normalize(arc[i + 1] - arc[i - 1]);
        T.y = 0.0f;
        if (glm::length(T) > 1e-4f) T = glm::normalize(T);
        else T = glm::vec3(1.0f, 0.0f, 0.0f);

        // In CCW perimeter traversal, interior is to the LEFT of T in the XZ plane: (-T.z, 0, T.x)
        glm::vec3 inDir(-T.z, 0.0f, T.x);

        // Outer curb vertex (u = 0.0, roadAttr.z = 1.0f)
        Vertex vOuter = {};
        vOuter.position[0] = pt.x; vOuter.position[1] = pt.y + 0.005f; vOuter.position[2] = pt.z;
        vOuter.normal[0] = 0.0f;   vOuter.normal[1] = 1.0f;            vOuter.normal[2] = 0.0f;
        vOuter.texCoords[0] = 0.0f; vOuter.texCoords[1] = accDist;
        vOuter.tangent[0] = 0.0f;  vOuter.tangent[1] = 0.0f;           vOuter.tangent[2] = 1.0f;
        outVerts.push_back(vOuter);

        // Inner edge vertex (u = 1.0)
        glm::vec3 ptIn = pt + inDir * wS;
        Vertex vInner = {};
        vInner.position[0] = ptIn.x; vInner.position[1] = ptIn.y + 0.005f; vInner.position[2] = ptIn.z;
        vInner.normal[0] = 0.0f;     vInner.normal[1] = 1.0f;              vInner.normal[2] = 0.0f;
        vInner.texCoords[0] = 1.0f;  vInner.texCoords[1] = accDist;
        vInner.tangent[0] = 0.0f;    vInner.tangent[1] = 0.0f;             vInner.tangent[2] = 1.0f;
        outVerts.push_back(vInner);
    }

    for (size_t i = 0; i + 1 < arc.size(); ++i) {
        uint32_t i0 = base + (uint32_t)(i * 2 + 0);
        uint32_t i1 = base + (uint32_t)(i * 2 + 1);
        uint32_t i2 = base + (uint32_t)((i + 1) * 2 + 0);
        uint32_t i3 = base + (uint32_t)((i + 1) * 2 + 1);

        outIndices.push_back(i0);
        outIndices.push_back(i3);
        outIndices.push_back(i2);

        outIndices.push_back(i0);
        outIndices.push_back(i1);
        outIndices.push_back(i3);
    }
}

static std::vector<glm::vec3> generateHermiteCurve(const glm::vec3& P0, const glm::vec3& T0,
                                                   const glm::vec3& P1, const glm::vec3& T1,
                                                   int steps = 6) {
    std::vector<glm::vec3> pts;
    pts.reserve(steps + 1);
    float L = glm::distance(glm::vec2(P0.x, P0.z), glm::vec2(P1.x, P1.z));
    if (L < 1e-4f) {
        pts.push_back(P0);
        pts.push_back(P1);
        return pts;
    }
    float scale = L * 0.5f;
    glm::vec3 m0 = T0 * scale;
    glm::vec3 m1 = T1 * scale;

    for (int s = 0; s <= steps; ++s) {
        float u = (float)s / (float)steps;
        float u2 = u * u;
        float u3 = u2 * u;
        float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
        float h10 = u3 - 2.0f * u2 + u;
        float h01 = -2.0f * u3 + 3.0f * u2;
        float h11 = u3 - u2;
        glm::vec3 p = h00 * P0 + h10 * m0 + h01 * P1 + h11 * m1;
        pts.push_back(p);
    }
    return pts;
}

static void addPerimeterPoint(std::vector<glm::vec3>& perim, const glm::vec3& pt) {
    if (perim.empty()) {
        perim.push_back(pt);
        return;
    }
    if (glm::distance(glm::vec2(pt.x, pt.z), glm::vec2(perim.back().x, perim.back().z)) < 0.02f) {
        return;
    }
    if (perim.size() >= 2) {
        const auto& p0 = perim[perim.size() - 2];
        const auto& p1 = perim[perim.size() - 1];
        glm::vec2 d0(p1.x - p0.x, p1.z - p0.z);
        glm::vec2 d1(pt.x - p1.x, pt.z - p1.z);
        float len0 = glm::length(d0);
        float len1 = glm::length(d1);
        if (len0 > 1e-4f && len1 > 1e-4f) {
            float cross = (d0.x * d1.y - d0.y * d1.x) / (len0 * len1);
            float dot = (d0.x * d1.x + d0.y * d1.y) / (len0 * len1);
            if (std::abs(cross) < 1e-4f && dot > 0.0f) {
                perim.back() = pt;
                return;
            }
        }
    }
    perim.push_back(pt);
}

static void triangulatePolygon2D(std::vector<glm::vec3> perimeter,
                                 std::vector<Vertex>& outVerts,
                                 std::vector<uint32_t>& outIndices) {
    size_t P = perimeter.size();
    if (P < 3) return;

    // Filter out collinear redundant vertices
    if (P > 3) {
        std::vector<glm::vec3> cleaned;
        cleaned.reserve(P);
        for (size_t i = 0; i < P; ++i) {
            const auto& prev = perimeter[(i + P - 1) % P];
            const auto& cur = perimeter[i];
            const auto& next = perimeter[(i + 1) % P];
            glm::vec2 d0(cur.x - prev.x, cur.z - prev.z);
            glm::vec2 d1(next.x - cur.x, next.z - cur.z);
            float l0 = glm::length(d0), l1 = glm::length(d1);
            if (l0 > 1e-4f && l1 > 1e-4f) {
                float cross = (d0.x * d1.y - d0.y * d1.x) / (l0 * l1);
                float dot = (d0.x * d1.x + d0.y * d1.y) / (l0 * l1);
                if (std::abs(cross) < 1e-4f && dot > 0.0f) {
                    continue;
                }
            }
            cleaned.push_back(cur);
        }
        if (cleaned.size() >= 3) {
            perimeter = std::move(cleaned);
            P = perimeter.size();
        }
    }

    auto cross2D = [](glm::vec2 v1, glm::vec2 v2) {
        return v1.x * v2.y - v1.y * v2.x;
    };

    auto isPointInTriangle = [&](glm::vec2 p, glm::vec2 a, glm::vec2 b, glm::vec2 c) -> bool {
        float cp1 = cross2D(b - a, p - a);
        float cp2 = cross2D(c - b, p - b);
        float cp3 = cross2D(a - c, p - c);
        return (cp1 >= -1e-5f && cp2 >= -1e-5f && cp3 >= -1e-5f) ||
               (cp1 <= 1e-5f && cp2 <= 1e-5f && cp3 <= 1e-5f);
    };

    // Compute 2D signed area to ensure counter-clockwise orientation
    float signedArea = 0.0f;
    for (size_t i = 0; i < P; ++i) {
        const auto& p1 = perimeter[i];
        const auto& p2 = perimeter[(i + 1) % P];
        signedArea += (p1.x * p2.z - p2.x * p1.z);
    }

    std::vector<uint32_t> V(P);
    for (size_t i = 0; i < P; ++i) V[i] = (uint32_t)i;
    if (signedArea > 0.0f) {
        std::reverse(V.begin(), V.end());
    }

    uint32_t baseVertex = (uint32_t)outVerts.size();
    for (const auto& pt : perimeter) {
        Vertex vp = {};
        vp.position[0] = pt.x; vp.position[1] = pt.y; vp.position[2] = pt.z;
        vp.normal[0] = 0.0f; vp.normal[1] = 1.0f; vp.normal[2] = 0.0f;
        vp.texCoords[0] = pt.x * 0.1f; vp.texCoords[1] = pt.z * 0.1f;
        vp.tangent[0] = 0.0f; vp.tangent[1] = 0.0f; vp.tangent[2] = 0.0f; // Pure asphalt
        outVerts.push_back(vp);
    }

    int count = (int)P;
    int maxIters = count * 3;
    int iter = 0;
    while (count > 2 && iter++ < maxIters) {
        bool earFound = false;
        for (int i = 0; i < count; ++i) {
            int prevIdx = (i - 1 + count) % count;
            int nextIdx = (i + 1) % count;

            uint32_t u = V[prevIdx];
            uint32_t v = V[i];
            uint32_t w = V[nextIdx];

            glm::vec2 a(perimeter[u].x, perimeter[u].z);
            glm::vec2 b(perimeter[v].x, perimeter[v].z);
            glm::vec2 c(perimeter[w].x, perimeter[w].z);

            float cp = cross2D(b - a, c - b);
            if (cp >= -1e-4f) continue; // Reflex angle or collinear

            bool hasPointInside = false;
            for (int j = 0; j < count; ++j) {
                if (j == prevIdx || j == i || j == nextIdx) continue;
                glm::vec2 p(perimeter[V[j]].x, perimeter[V[j]].z);
                if (isPointInTriangle(p, a, b, c)) {
                    hasPointInside = true;
                    break;
                }
            }

            if (!hasPointInside) {
                glm::vec3 norm = glm::cross(perimeter[v] - perimeter[u], perimeter[w] - perimeter[u]);
                if (glm::length(norm) > 1e-4f) {
                    if (norm.y < 0.0f) {
                        outIndices.push_back(baseVertex + u);
                        outIndices.push_back(baseVertex + w);
                        outIndices.push_back(baseVertex + v);
                    } else {
                        outIndices.push_back(baseVertex + u);
                        outIndices.push_back(baseVertex + v);
                        outIndices.push_back(baseVertex + w);
                    }
                }
                V.erase(V.begin() + i);
                count--;
                earFound = true;
                break;
            }
        }

        if (!earFound) {
            for (int i = 1; i + 1 < count; ++i) {
                uint32_t u = V[0], v = V[i], w = V[i + 1];
                glm::vec3 norm = glm::cross(perimeter[v] - perimeter[u], perimeter[w] - perimeter[u]);
                if (glm::length(norm) < 1e-4f) continue;
                if (norm.y < 0.0f) {
                    outIndices.push_back(baseVertex + u);
                    outIndices.push_back(baseVertex + w);
                    outIndices.push_back(baseVertex + v);
                } else {
                    outIndices.push_back(baseVertex + u);
                    outIndices.push_back(baseVertex + v);
                    outIndices.push_back(baseVertex + w);
                }
            }
            break;
        }
    }
}

void RoadGenerator::generateJunctionPatches(std::vector<ProcessedRoadSegment>& segments,
                                            std::vector<RoadJunction>& junctions,
                                            const glm::dvec3* optChunkCenterECEF) {
    // 1. Reset all segment setbacks
    for (auto& seg : segments) {
        seg.startSetback = 0.0f;
        seg.endSetback = 0.0f;
    }

    // 2. Compute setbacks for each multi-road junction
    for (auto& junc : junctions) {
        junc.patchVertices.clear();
        junc.patchIndices.clear();
        junc.patchMesh = nullptr;

        if (junc.connections.size() < 2) continue;

        struct ConnInfo {
            int connIdx;
            int segmentIdx;
            bool isStart;
            glm::vec2 dirAway;
            float angle;
            float width;
            float length;
        };

        std::vector<ConnInfo> connInfos;
        for (size_t c = 0; c < junc.connections.size(); ++c) {
            const auto& conn = junc.connections[c];
            if (conn.segmentIdx < 0 || conn.segmentIdx >= (int)segments.size()) continue;
            const auto& seg = segments[conn.segmentIdx];
            if (seg.splinePoints.size() < 2) continue;

            glm::vec3 pAway(0.0f);
            if (conn.isStart) {
                size_t sampleIdx = std::min((size_t)4, seg.splinePoints.size() - 1);
                pAway = seg.splinePoints[sampleIdx].pos;
            } else {
                size_t sampleIdx = (seg.splinePoints.size() > 4) ? (seg.splinePoints.size() - 5) : 0;
                pAway = seg.splinePoints[sampleIdx].pos;
            }

            glm::vec2 d(pAway.x - junc.position.x, pAway.z - junc.position.z);
            float len = glm::length(d);
            if (len > 1e-4f) d /= len;
            else d = glm::vec2(1.0f, 0.0f);

            float ang = std::atan2(d.y, d.x);
            connInfos.push_back({ (int)c, conn.segmentIdx, conn.isStart, d, ang, seg.totalWidth, seg.length });
        }

        if (connInfos.size() < 2) continue;

        // 2-way transitions:
        if (connInfos.size() == 2) {
            float dotP = glm::dot(connInfos[0].dirAway, connInfos[1].dirAway);
            float widthDiff = std::abs(connInfos[0].width - connInfos[1].width);

            // If truly identical road continuing seamlessly (same width and collinear), 0 setback
            if (widthDiff < 0.2f && dotP < -0.995f) {
                continue;
            }

            // Otherwise, this is a lane taper (width transition) or an angle bend:
            // Snip both road ends so they don't form a blunt cliff or folded corner,
            // and generate a smooth transition curve!
            float taperDist = (widthDiff >= 0.2f) ? std::clamp(widthDiff * 3.0f + 6.0f, 8.0f, 25.0f) : 0.0f;
            float bendDist = 0.0f;
            if (dotP >= -0.995f) {
                float angle = std::acos(std::clamp(dotP, -1.0f, 1.0f));
                float curveRadius = std::max(16.0f, std::max(connInfos[0].width, connInfos[1].width) * 2.0f);
                bendDist = curveRadius / std::tan(std::max(0.20f, angle * 0.5f));
            }
            float neededSetback = std::max(taperDist, bendDist);
            float sb0 = std::clamp(neededSetback, 4.0f, connInfos[0].length * 0.40f);
            float sb1 = std::clamp(neededSetback, 4.0f, connInfos[1].length * 0.40f);

            if (connInfos[0].isStart) segments[connInfos[0].segmentIdx].startSetback = std::max(segments[connInfos[0].segmentIdx].startSetback, sb0);
            else segments[connInfos[0].segmentIdx].endSetback = std::max(segments[connInfos[0].segmentIdx].endSetback, sb0);

            if (connInfos[1].isStart) segments[connInfos[1].segmentIdx].startSetback = std::max(segments[connInfos[1].segmentIdx].startSetback, sb1);
            else segments[connInfos[1].segmentIdx].endSetback = std::max(segments[connInfos[1].segmentIdx].endSetback, sb1);

            continue;
        }

        // Sort connections by angle counter-clockwise around junction center
        std::sort(connInfos.begin(), connInfos.end(), [](const ConnInfo& a, const ConnInfo& b){
            return a.angle < b.angle;
        });

        // Compute setback for each road meeting at this junction
        size_t M = connInfos.size();
        for (size_t i = 0; i < M; ++i) {
            auto& ci = connInfos[i];
            const auto& prev = connInfos[(i + M - 1) % M];
            const auto& next = connInfos[(i + 1) % M];

            auto angleBetween = [](float a1, float a2) {
                float diff = std::abs(a1 - a2);
                if (diff > (float)M_PI) diff = 2.0f * (float)M_PI - diff;
                return diff;
            };

            float diffPrev = angleBetween(ci.angle, prev.angle);
            float diffNext = angleBetween(ci.angle, next.angle);

            float neededPrev = 0.0f;
            if (diffPrev < 2.75f) {
                float sinHalf = std::sin(std::max(0.08f, diffPrev * 0.5f));
                neededPrev = ((ci.width + prev.width) * 0.5f) / (2.0f * sinHalf) + 2.0f;
            } else {
                float wDiff = std::abs(ci.width - prev.width);
                neededPrev = (wDiff >= 0.2f) ? std::clamp(wDiff * 2.0f, 4.0f, 10.0f) : 0.0f;
            }

            float neededNext = 0.0f;
            if (diffNext < 2.75f) {
                float sinHalf = std::sin(std::max(0.08f, diffNext * 0.5f));
                neededNext = ((ci.width + next.width) * 0.5f) / (2.0f * sinHalf) + 2.0f;
            } else {
                float wDiff = std::abs(ci.width - next.width);
                neededNext = (wDiff >= 0.2f) ? std::clamp(wDiff * 2.0f, 4.0f, 10.0f) : 0.0f;
            }

            float neededSetback = std::max(neededPrev, neededNext);
            float maxSetback = std::max(4.0f, std::min(60.0f, ci.length * 0.35f));
            float minSetback = std::min(maxSetback, (neededSetback > 0.1f) ? std::max(4.0f, ci.width * 0.35f) : 0.0f);
            float setback = std::clamp(neededSetback, minSetback, maxSetback);

            if (ci.isStart) {
                segments[ci.segmentIdx].startSetback = std::max(segments[ci.segmentIdx].startSetback, setback);
            } else {
                segments[ci.segmentIdx].endSetback = std::max(segments[ci.segmentIdx].endSetback, setback);
            }
        }
    }

    // 3. Generate junction patch plates for all multi-road junctions
    for (auto& junc : junctions) {
        if (junc.connections.size() < 2) continue;

        // --------------------------------------------------------------------
        // SPECIAL CASE: 2-Way Junction (Smooth Hermite Ribbon Transition Curve)
        // --------------------------------------------------------------------
        if (junc.connections.size() == 2) {
            const auto& conn0 = junc.connections[0];
            const auto& conn1 = junc.connections[1];
            if (conn0.segmentIdx < 0 || conn0.segmentIdx >= (int)segments.size() ||
                conn1.segmentIdx < 0 || conn1.segmentIdx >= (int)segments.size()) continue;

            const auto& seg0 = segments[conn0.segmentIdx];
            const auto& seg1 = segments[conn1.segmentIdx];
            if (seg0.splinePoints.size() < 2 || seg1.splinePoints.size() < 2) continue;

            float sb0 = conn0.isStart ? seg0.startSetback : seg0.endSetback;
            float sb1 = conn1.isStart ? seg1.startSetback : seg1.endSetback;

            // If no setback was needed (identical straight road), skip
            if (sb0 <= 0.01f && sb1 <= 0.01f) continue;

            auto trimmed0 = trimSpline(seg0.splinePoints, seg0.startSetback, seg0.endSetback);
            auto trimmed1 = trimSpline(seg1.splinePoints, seg1.startSetback, seg1.endSetback);
            if (trimmed0.size() < 2 || trimmed1.size() < 2) continue;

            const auto& cap0 = conn0.isStart ? trimmed0.front() : trimmed0.back();
            const auto& cap1 = conn1.isStart ? trimmed1.front() : trimmed1.back();

            // Tangent at P0 pointing INTO the junction (toward P1)
            glm::vec3 P0 = cap0.pos;
            glm::vec3 t0 = conn0.isStart ? -cap0.tangent : cap0.tangent;
            t0.y = 0.0f;
            if (glm::length(t0) > 1e-4f) t0 = glm::normalize(t0);

            // Tangent at P1 pointing AWAY from the junction (along seg1)
            glm::vec3 P1 = cap1.pos;
            glm::vec3 t1 = conn1.isStart ? cap1.tangent : -cap1.tangent;
            t1.y = 0.0f;
            if (glm::length(t1) > 1e-4f) t1 = glm::normalize(t1);

            glm::vec3 chord = P1 - P0;
            chord.y = 0.0f;
            float L = glm::length(chord);
            if (L < 1e-3f) continue;

            // Ensure tangents point forward along the chord direction
            if (glm::dot(t0, chord) < 0.0f) t0 = -t0;
            if (glm::dot(t1, chord) < 0.0f) t1 = -t1;

            float scale = L * 0.75f;
            glm::vec3 m0 = t0 * scale;
            glm::vec3 m1 = t1 * scale;

            int steps = std::max(6, std::min(24, (int)std::ceil(L / 1.5f)));

            bool sameLanes = (seg0.lanes == seg1.lanes && seg0.oneway == seg1.oneway);
            bool isLaneDiffOne = (!sameLanes && std::abs(seg0.lanes - seg1.lanes) == 1);

            uint32_t baseVertex = (uint32_t)junc.patchVertices.size();
            float vAcc = cap0.distance;
            glm::vec3 lastCenter = P0;

            if (sameLanes) {
                float lanesVal = (float)seg0.lanes;
                float onewayVal = seg0.oneway ? 1.0f : 0.0f;

                for (int s = 0; s <= steps; ++s) {
                    float u_t = (float)s / (float)steps;
                    float sFactor = u_t * u_t * (3.0f - 2.0f * u_t);

                    float u2 = u_t * u_t;
                    float u3 = u2 * u_t;
                    float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
                    float h10 = u3 - 2.0f * u2 + u_t;
                    float h01 = -2.0f * u3 + 3.0f * u2;
                    float h11 = u3 - u2;

                    glm::vec3 center = h00 * P0 + h10 * m0 + h01 * P1 + h11 * m1;
                    if (s > 0) vAcc += glm::distance(center, lastCenter);
                    lastCenter = center;

                    float dh00 = 6.0f * u2 - 6.0f * u_t;
                    float dh10 = 3.0f * u2 - 4.0f * u_t + 1.0f;
                    float dh01 = -6.0f * u2 + 6.0f * u_t;
                    float dh11 = 3.0f * u2 - 2.0f * u_t;

                    glm::vec3 deriv = dh00 * P0 + dh10 * m0 + dh01 * P1 + dh11 * m1;
                    deriv.y = 0.0f;
                    glm::vec3 T = (glm::length(deriv) > 1e-4f) ? glm::normalize(deriv) : glm::normalize(chord);
                    glm::vec3 N(0.0f, 1.0f, 0.0f);
                    glm::vec3 R = glm::normalize(glm::cross(T, N));

                    // Smooth S-curve width & shoulder interpolation (Page 1)
                    float w = glm::mix(seg0.totalWidth, seg1.totalWidth, sFactor);
                    float sW = glm::mix(seg0.shoulderWidth, seg1.shoulderWidth, sFactor);
                    float shoulderRatio = (w > 0.001f) ? std::clamp(sW / w, 0.0f, 0.35f) : 0.05f;

                    glm::vec3 pL = center - R * (w * 0.5f);
                    glm::vec3 pR = center + R * (w * 0.5f);

                    Vertex vL = {};
                    vL.position[0] = pL.x; vL.position[1] = pL.y; vL.position[2] = pL.z;
                    vL.normal[0] = 0.0f; vL.normal[1] = 1.0f; vL.normal[2] = 0.0f;
                    vL.texCoords[0] = 0.0f; vL.texCoords[1] = vAcc;
                    vL.tangent[0] = lanesVal; vL.tangent[1] = onewayVal; vL.tangent[2] = shoulderRatio;
                    junc.patchVertices.push_back(vL);

                    Vertex vR = {};
                    vR.position[0] = pR.x; vR.position[1] = pR.y; vR.position[2] = pR.z;
                    vR.normal[0] = 0.0f; vR.normal[1] = 1.0f; vR.normal[2] = 0.0f;
                    vR.texCoords[0] = 1.0f; vR.texCoords[1] = vAcc;
                    vR.tangent[0] = lanesVal; vR.tangent[1] = onewayVal; vR.tangent[2] = shoulderRatio;
                    junc.patchVertices.push_back(vR);
                }

                for (int s = 0; s < steps; ++s) {
                    uint32_t i0 = baseVertex + (uint32_t)(s * 2 + 0);
                    uint32_t i1 = baseVertex + (uint32_t)(s * 2 + 1);
                    uint32_t i2 = baseVertex + (uint32_t)((s + 1) * 2 + 0);
                    uint32_t i3 = baseVertex + (uint32_t)((s + 1) * 2 + 1);

                    junc.patchIndices.push_back(i0);
                    junc.patchIndices.push_back(i3);
                    junc.patchIndices.push_back(i2);

                    junc.patchIndices.push_back(i0);
                    junc.patchIndices.push_back(i1);
                    junc.patchIndices.push_back(i3);
                }
            } else if (isLaneDiffOne) {
                // Lane Merge / Drop Taper (Page 1: lane count difference = 1, fallback: rightmost lane)
                int nMax = std::max(seg0.lanes, seg1.lanes);
                int nMin = std::min(seg0.lanes, seg1.lanes);
                bool seg0Wider = (seg0.lanes > seg1.lanes);

                float lWidth = std::max(seg0.laneWidth, seg1.laneWidth);
                float wThrough = (float)nMin * lWidth;

                for (int s = 0; s <= steps; ++s) {
                    float u_t = (float)s / (float)steps;
                    float sFactor = u_t * u_t * (3.0f - 2.0f * u_t);

                    float u2 = u_t * u_t;
                    float u3 = u2 * u_t;
                    float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
                    float h10 = u3 - 2.0f * u2 + u_t;
                    float h01 = -2.0f * u3 + 3.0f * u2;
                    float h11 = u3 - u2;

                    glm::vec3 center = h00 * P0 + h10 * m0 + h01 * P1 + h11 * m1;
                    if (s > 0) vAcc += glm::distance(center, lastCenter);
                    lastCenter = center;

                    float dh00 = 6.0f * u2 - 6.0f * u_t;
                    float dh10 = 3.0f * u2 - 4.0f * u_t + 1.0f;
                    float dh01 = -6.0f * u2 + 6.0f * u_t;
                    float dh11 = 3.0f * u2 - 2.0f * u_t;

                    glm::vec3 deriv = dh00 * P0 + dh10 * m0 + dh01 * P1 + dh11 * m1;
                    deriv.y = 0.0f;
                    glm::vec3 T = (glm::length(deriv) > 1e-4f) ? glm::normalize(deriv) : glm::normalize(chord);
                    glm::vec3 N(0.0f, 1.0f, 0.0f);
                    glm::vec3 R = glm::normalize(glm::cross(T, N));

                    float taper = seg0Wider ? (1.0f - sFactor) : sFactor;
                    float wTaper = lWidth * taper;
                    float sW = glm::mix(seg0.shoulderWidth, seg1.shoulderWidth, sFactor);

                    float currentTotalW = sW + wThrough + wTaper + sW;
                    float shoulderRatio = (currentTotalW > 0.001f) ? std::clamp(sW / currentTotalW, 0.0f, 0.35f) : 0.05f;

                    glm::vec3 pL = center - R * (currentTotalW * 0.5f);
                    glm::vec3 pDiv = pL + R * (sW + wThrough);
                    glm::vec3 pR = center + R * (currentTotalW * 0.5f);

                    float divU = (nMax > 0) ? ((float)nMin / (float)nMax) : 0.5f;

                    // Vertex 0: Left edge (u = 0.0)
                    Vertex vL = {};
                    vL.position[0] = pL.x; vL.position[1] = pL.y; vL.position[2] = pL.z;
                    vL.normal[0] = 0.0f; vL.normal[1] = 1.0f; vL.normal[2] = 0.0f;
                    vL.texCoords[0] = 0.0f; vL.texCoords[1] = vAcc;
                    vL.tangent[0] = (float)nMax; vL.tangent[1] = 1.0f; vL.tangent[2] = shoulderRatio;
                    junc.patchVertices.push_back(vL);

                    // Vertex 1: Pinned divider between through lanes and tapering lane
                    Vertex vD = {};
                    vD.position[0] = pDiv.x; vD.position[1] = pDiv.y; vD.position[2] = pDiv.z;
                    vD.normal[0] = 0.0f; vD.normal[1] = 1.0f; vD.normal[2] = 0.0f;
                    vD.texCoords[0] = divU; vD.texCoords[1] = vAcc;
                    vD.tangent[0] = (float)nMax; vD.tangent[1] = 1.0f; vD.tangent[2] = shoulderRatio;
                    junc.patchVertices.push_back(vD);

                    // Vertex 2: Right edge (u = 1.0)
                    Vertex vR = {};
                    vR.position[0] = pR.x; vR.position[1] = pR.y; vR.position[2] = pR.z;
                    vR.normal[0] = 0.0f; vR.normal[1] = 1.0f; vR.normal[2] = 0.0f;
                    vR.texCoords[0] = 1.0f; vR.texCoords[1] = vAcc;
                    vR.tangent[0] = (float)nMax; vR.tangent[1] = 1.0f; vR.tangent[2] = shoulderRatio;
                    junc.patchVertices.push_back(vR);
                }

                for (int s = 0; s < steps; ++s) {
                    uint32_t i0 = baseVertex + (uint32_t)(s * 3 + 0);
                    uint32_t i1 = baseVertex + (uint32_t)(s * 3 + 1);
                    uint32_t i2 = baseVertex + (uint32_t)(s * 3 + 2);
                    uint32_t j0 = baseVertex + (uint32_t)((s + 1) * 3 + 0);
                    uint32_t j1 = baseVertex + (uint32_t)((s + 1) * 3 + 1);
                    uint32_t j2 = baseVertex + (uint32_t)((s + 1) * 3 + 2);

                    junc.patchIndices.push_back(i0);
                    junc.patchIndices.push_back(j1);
                    junc.patchIndices.push_back(j0);

                    junc.patchIndices.push_back(i0);
                    junc.patchIndices.push_back(i1);
                    junc.patchIndices.push_back(j1);

                    junc.patchIndices.push_back(i1);
                    junc.patchIndices.push_back(j2);
                    junc.patchIndices.push_back(j1);

                    junc.patchIndices.push_back(i1);
                    junc.patchIndices.push_back(i2);
                    junc.patchIndices.push_back(j2);
                }
            } else {
                for (int s = 0; s <= steps; ++s) {
                    float u_t = (float)s / (float)steps;
                    float sFactor = u_t * u_t * (3.0f - 2.0f * u_t);

                    float u2 = u_t * u_t;
                    float u3 = u2 * u_t;
                    float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
                    float h10 = u3 - 2.0f * u2 + u_t;
                    float h01 = -2.0f * u3 + 3.0f * u2;
                    float h11 = u3 - u2;

                    glm::vec3 center = h00 * P0 + h10 * m0 + h01 * P1 + h11 * m1;
                    if (s > 0) vAcc += glm::distance(center, lastCenter);
                    lastCenter = center;

                    float dh00 = 6.0f * u2 - 6.0f * u_t;
                    float dh10 = 3.0f * u2 - 4.0f * u_t + 1.0f;
                    float dh01 = -6.0f * u2 + 6.0f * u_t;
                    float dh11 = 3.0f * u2 - 2.0f * u_t;

                    glm::vec3 deriv = dh00 * P0 + dh10 * m0 + dh01 * P1 + dh11 * m1;
                    deriv.y = 0.0f;
                    glm::vec3 T = (glm::length(deriv) > 1e-4f) ? glm::normalize(deriv) : glm::normalize(chord);
                    glm::vec3 N(0.0f, 1.0f, 0.0f);
                    glm::vec3 R = glm::normalize(glm::cross(T, N));

                    float w = glm::mix(seg0.totalWidth, seg1.totalWidth, sFactor);
                    glm::vec3 pL = center - R * (w * 0.5f);
                    glm::vec3 pR = center + R * (w * 0.5f);

                    Vertex vL = {};
                    vL.position[0] = pL.x; vL.position[1] = pL.y; vL.position[2] = pL.z;
                    vL.normal[0] = 0.0f; vL.normal[1] = 1.0f; vL.normal[2] = 0.0f;
                    vL.texCoords[0] = 0.0f; vL.texCoords[1] = vAcc;
                    vL.tangent[0] = 0.0f; vL.tangent[1] = 0.0f; vL.tangent[2] = 0.0f;
                    junc.patchVertices.push_back(vL);

                    Vertex vR = {};
                    vR.position[0] = pR.x; vR.position[1] = pR.y; vR.position[2] = pR.z;
                    vR.normal[0] = 0.0f; vR.normal[1] = 1.0f; vR.normal[2] = 0.0f;
                    vR.texCoords[0] = 1.0f; vR.texCoords[1] = vAcc;
                    vR.tangent[0] = 0.0f; vR.tangent[1] = 0.0f; vR.tangent[2] = 0.0f;
                    junc.patchVertices.push_back(vR);
                }

                for (int s = 0; s < steps; ++s) {
                    uint32_t i0 = baseVertex + (uint32_t)(s * 2 + 0);
                    uint32_t i1 = baseVertex + (uint32_t)(s * 2 + 1);
                    uint32_t i2 = baseVertex + (uint32_t)((s + 1) * 2 + 0);
                    uint32_t i3 = baseVertex + (uint32_t)((s + 1) * 2 + 1);

                    junc.patchIndices.push_back(i0);
                    junc.patchIndices.push_back(i3);
                    junc.patchIndices.push_back(i2);

                    junc.patchIndices.push_back(i0);
                    junc.patchIndices.push_back(i1);
                    junc.patchIndices.push_back(i3);
                }
            }

            if (!junc.patchVertices.empty() && !junc.patchIndices.empty()) {
                junc.patchMesh = std::make_shared<Mesh>(junc.patchVertices.data(), (Uint32)junc.patchVertices.size(),
                                                        junc.patchIndices.data(), (Uint32)junc.patchIndices.size());
            }

            continue; // Skip multi-way ear-clipping loop for this 2-way junction
        }

        // --------------------------------------------------------------------
        // Multi-Way Junction Patch Plates (N >= 3)
        // Strict cyclic counter-clockwise angular ordering with physically accurate cap corners
        // --------------------------------------------------------------------
        struct SortedCap {
            int segIdx;
            float angle;
            glm::vec3 CW;
            glm::vec3 CCW;
            glm::vec3 dirAway;
            float shoulderW;
        };

        std::vector<SortedCap> sortedCaps;

        for (const auto& conn : junc.connections) {
            if (conn.segmentIdx < 0 || conn.segmentIdx >= (int)segments.size()) continue;
            const auto& seg = segments[conn.segmentIdx];
            if (seg.splinePoints.size() < 2) continue;

            auto trimmed = trimSpline(seg.splinePoints, seg.startSetback, seg.endSetback);
            if (trimmed.size() < 2) continue;

            const auto& pt = conn.isStart ? trimmed.front() : trimmed.back();
            glm::vec3 dAway = conn.isStart ? pt.tangent : -pt.tangent;
            dAway.y = 0.0f;
            if (glm::length(dAway) > 1e-4f) dAway = glm::normalize(dAway);
            else dAway = glm::vec3(1.0f, 0.0f, 0.0f);

            float ang = std::atan2(dAway.z, dAway.x);
            if (ang < 0.0f) ang += 2.0f * (float)M_PI;

            // Looking along dAway (away from junction center):
            // In XZ plane, counter-clockwise (to the left) is (-dAway.z, 0, dAway.x)
            glm::vec3 leftDir(-dAway.z, 0.0f, dAway.x);
            float halfW = seg.totalWidth * 0.5f;

            glm::vec3 CCW = pt.pos + leftDir * halfW;
            glm::vec3 CW  = pt.pos - leftDir * halfW;

            sortedCaps.push_back({ conn.segmentIdx, ang, CW, CCW, dAway, seg.shoulderWidth });
        }

        if (sortedCaps.size() < 2) continue;

        std::sort(sortedCaps.begin(), sortedCaps.end(), [](const SortedCap& a, const SortedCap& b){
            return a.angle < b.angle;
        });

        std::vector<glm::vec3> perimeter;
        perimeter.reserve(sortedCaps.size() * 10);

        size_t M = sortedCaps.size();
        for (size_t i = 0; i < M; ++i) {
            const auto& cur = sortedCaps[i];
            const auto& nxt = sortedCaps[(i + 1) % M];

            // 1. Across cur's cap from CW to CCW
            addPerimeterPoint(perimeter, cur.CW);
            addPerimeterPoint(perimeter, cur.CCW);

            // 2. Corner between cur.CCW and nxt.CW
            float dAngle = nxt.angle - cur.angle;
            if (dAngle < 0.0f) dAngle += 2.0f * (float)M_PI;

            float sW = std::clamp(std::max(cur.shoulderW, nxt.shoulderW), 1.2f, 2.5f);

            if (dAngle >= 2.75f) {
                // Through-road connection (opposite roads, angle ~ 180 deg)
                // Connect with smooth Hermite through curve carrying the through-shoulder across
                auto throughCurve = generateHermiteCurve(cur.CCW, -cur.dirAway, nxt.CW, nxt.dirAway, 6);
                generateCurbReturnRibbon(throughCurve, sW, junc.patchVertices, junc.patchIndices);
                for (size_t k = 1; k + 1 < throughCurve.size(); ++k) {
                    addPerimeterPoint(perimeter, throughCurve[k]);
                }
            } else {
                // Corner / Turn / Gore fillet
                glm::vec2 dirCur(-cur.dirAway.x, -cur.dirAway.z);
                glm::vec2 dirNxt(nxt.dirAway.x, nxt.dirAway.z);
                auto fillet = generateFilletArc(cur.CCW, nxt.CW, dirCur, dirNxt, 6);
                generateCurbReturnRibbon(fillet, sW, junc.patchVertices, junc.patchIndices);
                for (size_t k = 1; k + 1 < fillet.size(); ++k) {
                    addPerimeterPoint(perimeter, fillet[k]);
                }
            }
        }

        triangulatePolygon2D(perimeter, junc.patchVertices, junc.patchIndices);

        if (optChunkCenterECEF) {
            for (auto& v : junc.patchVertices) {
                glm::dvec3 ecef = *optChunkCenterECEF + glm::dvec3(v.position[0], v.position[1], v.position[2]);
                if (glm::length(ecef) > 1e-3) {
                    glm::vec3 n = glm::vec3(glm::normalize(ecef));
                    v.normal[0] = n.x; v.normal[1] = n.y; v.normal[2] = n.z;
                }
            }
        }

        if (!junc.patchVertices.empty() && !junc.patchIndices.empty()) {
            junc.patchMesh = std::make_shared<Mesh>(junc.patchVertices.data(), (Uint32)junc.patchVertices.size(),
                                                    junc.patchIndices.data(), (Uint32)junc.patchIndices.size());
        }
    }
}

std::shared_ptr<Mesh> RoadGenerator::buildMergedMesh(const std::vector<ProcessedRoadSegment>& segments,
                                                     const std::vector<RoadJunction>& junctions,
                                                     bool filterChunk,
                                                     float minX, float maxX,
                                                     float minZ, float maxZ) {
    std::vector<Vertex> mergedVerts;
    std::vector<uint32_t> mergedIdxs;

    // 1. Add trimmed road ribbons
    for (const auto& seg : segments) {
        if (filterChunk && !seg.in500mChunk) continue;
        auto trimmed = trimSpline(seg.splinePoints, seg.startSetback, seg.endSetback);
        std::vector<Vertex> segVerts;
        std::vector<uint32_t> segIdxs;
        generateRibbonGeometry(trimmed, seg.totalWidth, seg.lanes, seg.oneway, seg.shoulderWidth, segVerts, segIdxs);
        clipRibbonAgainstJunctions(segVerts, seg, junctions, segments);
        uint32_t baseVertex = (uint32_t)mergedVerts.size();
        for (const auto& v : segVerts) mergedVerts.push_back(v);
        for (auto idx : segIdxs) mergedIdxs.push_back(baseVertex + idx);
    }

    // 2. Add junction patch plates
    for (const auto& junc : junctions) {
        if (junc.patchVertices.empty() || junc.patchIndices.empty()) continue;
        if (filterChunk) {
            if (junc.position.x < minX - 1.0f || junc.position.x > maxX + 1.0f ||
                junc.position.z < minZ - 1.0f || junc.position.z > maxZ + 1.0f) {
                continue;
            }
        }

        uint32_t baseVertex = (uint32_t)mergedVerts.size();
        for (const auto& v : junc.patchVertices) {
            mergedVerts.push_back(v);
        }
        for (auto idx : junc.patchIndices) {
            mergedIdxs.push_back(baseVertex + idx);
        }
    }

    if (mergedVerts.empty() || mergedIdxs.empty()) return nullptr;

    return std::make_shared<Mesh>(mergedVerts.data(), (Uint32)mergedVerts.size(),
                                  mergedIdxs.data(), (Uint32)mergedIdxs.size());
}

void RoadGenerator::buildNetworkTopology(std::vector<ProcessedRoadSegment>& segments,
                                         std::vector<RoadJunction>& outJunctions,
                                         bool verbose) {
    outJunctions.clear();
    const float mergeDistSq = 0.50f * 0.50f; // 50cm tolerance for shared 3D coordinates

    for (size_t s = 0; s < segments.size(); ++s) {
        auto& seg = segments[s];
        if (seg.controlPoints.size() < 2) continue;

        seg.startJunctionId = -1;
        seg.endJunctionId = -1;

        // 1. Start endpoint (index 0)
        glm::vec3 pStart = seg.controlPoints.front();
        int startJunc = -1;
        for (size_t j = 0; j < outJunctions.size(); ++j) {
            glm::vec3 delta = outJunctions[j].position - pStart;
            if (glm::dot(delta, delta) <= mergeDistSq) {
                startJunc = (int)j;
                break;
            }
        }
        if (startJunc >= 0) {
            outJunctions[startJunc].connections.push_back({ (int)s, true });
            seg.startJunctionId = (int)outJunctions[startJunc].id;
        } else {
            RoadJunction newJunc;
            newJunc.id = (uint32_t)outJunctions.size();
            newJunc.position = pStart;
            newJunc.connections.push_back({ (int)s, true });
            seg.startJunctionId = (int)newJunc.id;
            outJunctions.push_back(newJunc);
        }

        // 2. End endpoint (last index)
        glm::vec3 pEnd = seg.controlPoints.back();
        int endJunc = -1;
        for (size_t j = 0; j < outJunctions.size(); ++j) {
            glm::vec3 delta = outJunctions[j].position - pEnd;
            if (glm::dot(delta, delta) <= mergeDistSq) {
                endJunc = (int)j;
                break;
            }
        }
        if (endJunc >= 0) {
            outJunctions[endJunc].connections.push_back({ (int)s, false });
            seg.endJunctionId = (int)outJunctions[endJunc].id;
        } else {
            RoadJunction newJunc;
            newJunc.id = (uint32_t)outJunctions.size();
            newJunc.position = pEnd;
            newJunc.connections.push_back({ (int)s, false });
            seg.endJunctionId = (int)newJunc.id;
            outJunctions.push_back(newJunc);
        }
    }

    if (verbose) {
        std::cout << "RoadGenerator: Built network topology - "
                  << outJunctions.size() << " junctions detected across "
                  << segments.size() << " segments" << std::endl;
    }
}

bool RoadGenerator::collapseShortJunctionSegments(std::vector<ProcessedRoadSegment>& segments, float minLength) {
    if (segments.empty()) return false;

    const float mergeCoordTol = 0.50f; // 50cm coordinate match for junction endpoint

    bool anyCollapsed = false;
    bool changed = true;
    int pass = 0;

    while (changed && pass < 10) {
        changed = false;
        pass++;

        int targetIdx = -1;

        for (size_t i = 0; i < segments.size(); ++i) {
            const auto& seg = segments[i];
            if (seg.controlPoints.size() < 2) continue;

            float len = seg.length;
            if (len <= 0.001f) {
                len = glm::distance(glm::vec2(seg.controlPoints.front().x, seg.controlPoints.front().z),
                                    glm::vec2(seg.controlPoints.back().x, seg.controlPoints.back().z));
            }
            if (len >= minLength) continue;

            glm::vec3 p0 = seg.controlPoints.front();
            glm::vec3 p1 = seg.controlPoints.back();

            // Count other segments meeting at p0
            int conns0 = 0;
            // Count other segments meeting at p1
            int conns1 = 0;

            for (size_t j = 0; j < segments.size(); ++j) {
                if (j == i) continue;
                const auto& other = segments[j];
                if (other.controlPoints.size() < 2) continue;

                glm::vec3 op0 = other.controlPoints.front();
                glm::vec3 op1 = other.controlPoints.back();

                if (glm::distance(p0, op0) <= mergeCoordTol || glm::distance(p0, op1) <= mergeCoordTol) {
                    conns0++;
                }
                if (glm::distance(p1, op0) <= mergeCoordTol || glm::distance(p1, op1) <= mergeCoordTol) {
                    conns1++;
                }
            }

            // Both ends must be multi-way junctions (>= 2 other segments meeting at each end)
            if (conns0 >= 2 && conns1 >= 2) {
                targetIdx = (int)i;
                break;
            }
        }

        if (targetIdx >= 0) {
            const auto& seg = segments[targetIdx];
            glm::vec3 p0 = seg.controlPoints.front();
            glm::vec3 p1 = seg.controlPoints.back();
            glm::vec3 mergedCenter = (p0 + p1) * 0.5f;

            // Redirect all other segments meeting at p0 or p1 to mergedCenter
            for (size_t j = 0; j < segments.size(); ++j) {
                if ((int)j == targetIdx) continue;
                auto& other = segments[j];
                if (other.controlPoints.size() < 2) continue;

                bool modified = false;
                if (glm::distance(p0, other.controlPoints.front()) <= mergeCoordTol ||
                    glm::distance(p1, other.controlPoints.front()) <= mergeCoordTol) {
                    other.controlPoints.front() = mergedCenter;
                    modified = true;
                }
                if (glm::distance(p0, other.controlPoints.back()) <= mergeCoordTol ||
                    glm::distance(p1, other.controlPoints.back()) <= mergeCoordTol) {
                    other.controlPoints.back() = mergedCenter;
                    modified = true;
                }

                if (modified) {
                    // Prune intermediate control points within 2.5m of mergedCenter
                    while (other.controlPoints.size() > 2 &&
                           glm::distance(glm::vec2(other.controlPoints[1].x, other.controlPoints[1].z),
                                         glm::vec2(mergedCenter.x, mergedCenter.z)) < 2.5f) {
                        other.controlPoints.erase(other.controlPoints.begin() + 1);
                    }
                    while (other.controlPoints.size() > 2 &&
                           glm::distance(glm::vec2(other.controlPoints[other.controlPoints.size() - 2].x,
                                                   other.controlPoints[other.controlPoints.size() - 2].z),
                                         glm::vec2(mergedCenter.x, mergedCenter.z)) < 2.5f) {
                        other.controlPoints.erase(other.controlPoints.end() - 2);
                    }
                    const glm::dvec3* optCenter = (other.chunkCenterECEF != glm::dvec3(0.0)) ? &other.chunkCenterECEF : nullptr;
                    other.splinePoints = generateSpline(other.controlPoints, other.sampleStep, optCenter);
                    if (!other.splinePoints.empty()) {
                        other.length = other.splinePoints.back().distance;
                    }
                }
            }

            segments.erase(segments.begin() + targetIdx);
            changed = true;
            anyCollapsed = true;
        }
    }

    return anyCollapsed;
}

void RoadGenerator::splitSegmentsAcrossGrid(const std::vector<ProcessedRoadSegment>& inSegments,
                                           float chunkSize,
                                           std::map<std::pair<int, int>, std::vector<ProcessedRoadSegment>>& outChunkSegments) {
    outChunkSegments.clear();
    if (inSegments.empty() || chunkSize <= 1.0f) return;

    float S = chunkSize;
    float halfS = S * 0.5f;

    uint32_t nextSubsegId = 1;

    for (const auto& origSeg : inSegments) {
        if (origSeg.controlPoints.size() < 2) continue;

        struct AugmentedPoint {
            glm::vec3 pos;
            bool isBoundary = false;
        };

        std::vector<AugmentedPoint> augPoints;
        augPoints.push_back({ origSeg.controlPoints[0], false });

        for (size_t i = 0; i + 1 < origSeg.controlPoints.size(); ++i) {
            glm::vec3 p0 = origSeg.controlPoints[i];
            glm::vec3 p1 = origSeg.controlPoints[i + 1];

            // Find all chunk boundary planes crossed between p0 and p1
            struct Crossing {
                float t;
                glm::vec3 pt;
            };
            std::vector<Crossing> crossings;

            // X planes: (k + 0.5f) * S
            float minX = std::min(p0.x, p1.x);
            float maxX = std::max(p0.x, p1.x);
            int kMinX = (int)std::ceil(minX / S - 0.5f);
            int kMaxX = (int)std::floor(maxX / S - 0.5f);
            if (std::abs(p1.x - p0.x) > 1e-4f) {
                for (int k = kMinX; k <= kMaxX; ++k) {
                    float xPlane = (k + 0.5f) * S;
                    float t = (xPlane - p0.x) / (p1.x - p0.x);
                    if (t > 1e-4f && t < 1.0f - 1e-4f) {
                        crossings.push_back({ t, glm::mix(p0, p1, t) });
                    }
                }
            }

            // Z planes: (k + 0.5f) * S
            float minZ = std::min(p0.z, p1.z);
            float maxZ = std::max(p0.z, p1.z);
            int kMinZ = (int)std::ceil(minZ / S - 0.5f);
            int kMaxZ = (int)std::floor(maxZ / S - 0.5f);
            if (std::abs(p1.z - p0.z) > 1e-4f) {
                for (int k = kMinZ; k <= kMaxZ; ++k) {
                    float zPlane = (k + 0.5f) * S;
                    float t = (zPlane - p0.z) / (p1.z - p0.z);
                    if (t > 1e-4f && t < 1.0f - 1e-4f) {
                        crossings.push_back({ t, glm::mix(p0, p1, t) });
                    }
                }
            }

            // Sort crossings by t ascending
            std::sort(crossings.begin(), crossings.end(), [](const Crossing& a, const Crossing& b) {
                return a.t < b.t;
            });

            // Deduplicate very close crossings
            for (size_t c = 0; c < crossings.size(); ++c) {
                if (c > 0 && std::abs(crossings[c].t - crossings[c - 1].t) < 1e-4f) continue;
                augPoints.push_back({ crossings[c].pt, true });
            }

            augPoints.push_back({ p1, false });
        }

        // Split augPoints into continuous subsegments bounded by boundary points
        size_t startIdx = 0;
        std::vector<ProcessedRoadSegment> segPieces;

        for (size_t i = 1; i < augPoints.size(); ++i) {
            if (augPoints[i].isBoundary || i == augPoints.size() - 1) {
                ProcessedRoadSegment sub = origSeg;
                sub.id = nextSubsegId++;
                sub.controlPoints.clear();
                for (size_t p = startIdx; p <= i; ++p) {
                    sub.controlPoints.push_back(augPoints[p].pos);
                }

                // Determine chunk coordinates from midpoint
                glm::vec3 mid = 0.5f * (sub.controlPoints.front() + sub.controlPoints.back());
                sub.chunkX = (int)std::floor((mid.x + halfS) / S);
                sub.chunkZ = (int)std::floor((mid.z + halfS) / S);

                // Regenerate spline points for subsegment
                sub.splinePoints = generateSpline(sub.controlPoints, sub.sampleStep);
                if (!sub.splinePoints.empty()) {
                    sub.length = sub.splinePoints.back().distance;
                }

                segPieces.push_back(sub);
                startIdx = i;
            }
        }

        // Link cross-chunk connections between consecutive pieces
        for (size_t p = 0; p + 1 < segPieces.size(); ++p) {
            auto& cur = segPieces[p];
            auto& nxt = segPieces[p + 1];

            glm::vec3 seamPt = cur.controlPoints.back();

            cur.endNeighbor.hasNeighbor = true;
            cur.endNeighbor.neighborChunkX = nxt.chunkX;
            cur.endNeighbor.neighborChunkZ = nxt.chunkZ;
            cur.endNeighbor.neighborSegmentId = nxt.id;
            cur.endNeighbor.neighborIsStart = true;
            cur.endNeighbor.boundaryPos = seamPt;

            nxt.startNeighbor.hasNeighbor = true;
            nxt.startNeighbor.neighborChunkX = cur.chunkX;
            nxt.startNeighbor.neighborChunkZ = cur.chunkZ;
            nxt.startNeighbor.neighborSegmentId = cur.id;
            nxt.startNeighbor.neighborIsStart = false;
            nxt.startNeighbor.boundaryPos = seamPt;
        }

        for (const auto& piece : segPieces) {
            outChunkSegments[{ piece.chunkX, piece.chunkZ }].push_back(piece);
        }
    }

    std::cout << "RoadGenerator: Sliced " << inSegments.size() << " road corridors into "
              << nextSubsegId - 1 << " subsegments across "
              << outChunkSegments.size() << " 512m chunks." << std::endl;
}

void RoadGenerator::splitSegmentsAcrossQuadsphere(
    const std::vector<ProcessedRoadSegment>& inSegments,
    uint16_t lod,
    std::map<std::tuple<uint8_t, uint32_t, uint32_t>, std::vector<ProcessedRoadSegment>>& outChunkSegments) {
    outChunkSegments.clear();
    if (inSegments.empty()) return;

    double tilesPerSide = static_cast<double>(1ULL << lod);
    double span = 2.0 / tilesPerSide;

    const double cLat = 40.592676;
    const double cLon = -74.515211;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double m_per_lat = 111139.0;
    const double m_per_lon = 111139.0 * std::cos(cLat * degToRad);

    uint32_t nextSubsegId = 1;

    for (const auto& origSeg : inSegments) {
        if (origSeg.controlPoints.size() < 2) continue;

        // Ensure we have GPS points for every control point
        std::vector<glm::dvec2> gpsList;
        gpsList.reserve(origSeg.controlPoints.size());

        for (size_t p = 0; p < origSeg.controlPoints.size(); ++p) {
            glm::dvec2 gps = (p < origSeg.gpsPoints.size()) ? origSeg.gpsPoints[p] : localPosToGPS(origSeg.controlPoints[p]);
            if (std::abs(gps.x) < 1e-4 && std::abs(gps.y) < 1e-4) {
                gps = localPosToGPS(origSeg.controlPoints[p]);
            }
            gpsList.push_back(gps);
        }

        struct QuadAugPoint {
            uint8_t face;
            double u, v;
            glm::dvec2 gps;
            float elev;
            glm::dvec3 ecef;
            bool isBoundary = false;
        };

        auto makeAugPoint = [&](const glm::dvec2& gps, float elev, bool isBnd) -> QuadAugPoint {
            QuadAugPoint ap;
            ap.gps = gps;
            ap.elev = elev;
            ap.isBoundary = isBnd;
            Quadsphere::geodeticToQuadsphere(gps.y, gps.x, ap.face, ap.u, ap.v);
            ap.ecef = Quadsphere::quadsphereToECEF(ap.face, ap.u, ap.v, elev);
            return ap;
        };

        std::vector<QuadAugPoint> augPoints;
        augPoints.reserve(origSeg.controlPoints.size() * 2);
        augPoints.push_back(makeAugPoint(gpsList[0], origSeg.controlPoints[0].y, false));

        for (size_t i = 0; i + 1 < origSeg.controlPoints.size(); ++i) {
            auto p0 = makeAugPoint(gpsList[i], origSeg.controlPoints[i].y, false);
            auto p1 = makeAugPoint(gpsList[i + 1], origSeg.controlPoints[i + 1].y, false);

            struct Crossing {
                double t;
                double u, v;
            };
            std::vector<Crossing> crossings;

            if (p0.face == p1.face) {
                // U boundary planes
                double minU = std::min(p0.u, p1.u);
                double maxU = std::max(p0.u, p1.u);
                int kMinU = static_cast<int>(std::ceil((minU + 1.0) / span));
                int kMaxU = static_cast<int>(std::floor((maxU + 1.0) / span));
                if (kMaxU - kMinU <= 10 && std::abs(p1.u - p0.u) > 1e-9) {
                    for (int k = kMinU; k <= kMaxU; ++k) {
                        double uPlane = -1.0 + k * span;
                        double t = (uPlane - p0.u) / (p1.u - p0.u);
                        if (t > 1e-5 && t < 1.0 - 1e-5) {
                            crossings.push_back({ t, uPlane, glm::mix(p0.v, p1.v, t) });
                        }
                    }
                }

                // V boundary planes
                double minV = std::min(p0.v, p1.v);
                double maxV = std::max(p0.v, p1.v);
                int kMinV = static_cast<int>(std::ceil((minV + 1.0) / span));
                int kMaxV = static_cast<int>(std::floor((maxV + 1.0) / span));
                if (kMaxV - kMinV <= 10 && std::abs(p1.v - p0.v) > 1e-9) {
                    for (int k = kMinV; k <= kMaxV; ++k) {
                        double vPlane = -1.0 + k * span;
                        double t = (vPlane - p0.v) / (p1.v - p0.v);
                        if (t > 1e-5 && t < 1.0 - 1e-5) {
                            crossings.push_back({ t, glm::mix(p0.u, p1.u, t), vPlane });
                        }
                    }
                }
            }

            // Sort crossings by t ascending
            std::sort(crossings.begin(), crossings.end(), [](const Crossing& a, const Crossing& b) {
                return a.t < b.t;
            });

            // Deduplicate very close crossings
            for (size_t c = 0; c < crossings.size(); ++c) {
                if (c > 0 && std::abs(crossings[c].t - crossings[c - 1].t) < 1e-5) continue;
                double t = crossings[c].t;
                glm::dvec2 crGPS = glm::mix(p0.gps, p1.gps, t);
                float crElev = glm::mix(p0.elev, p1.elev, static_cast<float>(t));
                QuadAugPoint crPt;
                crPt.face = p0.face;
                crPt.u = crossings[c].u;
                crPt.v = crossings[c].v;
                crPt.gps = crGPS;
                crPt.elev = crElev;
                crPt.ecef = Quadsphere::quadsphereToECEF(p0.face, crPt.u, crPt.v, crElev);
                crPt.isBoundary = true;
                augPoints.push_back(crPt);
            }

            augPoints.push_back(p1);
        }

        // Split augPoints into continuous subsegments bounded by boundary points
        size_t startIdx = 0;
        std::vector<ProcessedRoadSegment> segPieces;

        for (size_t i = 1; i < augPoints.size(); ++i) {
            if (augPoints[i].isBoundary || i == augPoints.size() - 1) {
                ProcessedRoadSegment sub = origSeg;
                sub.id = nextSubsegId++;
                sub.controlPoints.clear();
                sub.gpsPoints.clear();

                // Compute tile coordinates from midpoint
                size_t midIdx = startIdx + (i - startIdx) / 2;
                uint8_t chunkFace = augPoints[midIdx].face;
                double midU = 0.5 * (augPoints[startIdx].u + augPoints[i].u);
                double midV = 0.5 * (augPoints[startIdx].v + augPoints[i].v);

                int tx = std::clamp(static_cast<int>(std::floor((midU + 1.0) / span)), 0, (1 << lod) - 1);
                int ty = std::clamp(static_cast<int>(std::floor((midV + 1.0) / span)), 0, (1 << lod) - 1);

                double uCenter = -1.0 + (tx + 0.5) * span;
                double vCenter = -1.0 + (ty + 0.5) * span;
                glm::dvec3 chunkCenterECEF = Quadsphere::quadsphereToECEF(chunkFace, uCenter, vCenter, 0.0);

                sub.face = chunkFace;
                sub.chunkX = tx;
                sub.chunkZ = ty;
                sub.chunkCenterECEF = chunkCenterECEF;

                // Subdivide spans longer than 10m (SPHERE.md Section 5.1 step 3)
                for (size_t p = startIdx; p <= i; ++p) {
                    if (p > startIdx) {
                        const auto& prev = augPoints[p - 1];
                        const auto& curr = augPoints[p];
                        double chordLen = glm::length(curr.ecef - prev.ecef);
                        if (chordLen > 10.0) {
                            int subdiv = static_cast<int>(std::ceil(chordLen / 10.0));
                            if (subdiv > 100) subdiv = 100;
                            for (int s = 1; s < subdiv; ++s) {
                                double frac = static_cast<double>(s) / static_cast<double>(subdiv);
                                glm::dvec2 subGps = glm::mix(prev.gps, curr.gps, frac);
                                float subElev = glm::mix(prev.elev, curr.elev, static_cast<float>(frac));
                                glm::dvec3 subEcef = Quadsphere::geodeticToECEF(subGps.y, subGps.x, subElev);
                                sub.controlPoints.push_back(glm::vec3(subEcef - chunkCenterECEF));
                                sub.gpsPoints.push_back(subGps);
                            }
                        }
                    }
                    sub.controlPoints.push_back(glm::vec3(augPoints[p].ecef - chunkCenterECEF));
                    sub.gpsPoints.push_back(augPoints[p].gps);
                }

                // Generate spline points with true sphere normals and ECEF positions
                float step = (sub.sampleStep > 0.0f) ? sub.sampleStep : 2.50f;
                sub.splinePoints = generateSpline(sub.controlPoints, step, &chunkCenterECEF);
                if (!sub.splinePoints.empty()) {
                    sub.length = sub.splinePoints.back().distance;
                }

                segPieces.push_back(sub);
                startIdx = i;
            }
        }

        // Link cross-chunk connections between consecutive pieces
        for (size_t p = 0; p + 1 < segPieces.size(); ++p) {
            auto& cur = segPieces[p];
            auto& nxt = segPieces[p + 1];

            glm::dvec3 seamECEF = cur.chunkCenterECEF + glm::dvec3(cur.controlPoints.back());

            cur.endNeighbor.hasNeighbor = true;
            cur.endNeighbor.neighborFace = nxt.face;
            cur.endNeighbor.neighborChunkX = nxt.chunkX;
            cur.endNeighbor.neighborChunkZ = nxt.chunkZ;
            cur.endNeighbor.neighborSegmentId = nxt.id;
            cur.endNeighbor.neighborIsStart = true;
            cur.endNeighbor.boundaryPos = cur.controlPoints.back();
            cur.endNeighbor.boundaryPosECEF = seamECEF;

            nxt.startNeighbor.hasNeighbor = true;
            nxt.startNeighbor.neighborFace = cur.face;
            nxt.startNeighbor.neighborChunkX = cur.chunkX;
            nxt.startNeighbor.neighborChunkZ = cur.chunkZ;
            nxt.startNeighbor.neighborSegmentId = cur.id;
            nxt.startNeighbor.neighborIsStart = false;
            nxt.startNeighbor.boundaryPos = nxt.controlPoints.front();
            nxt.startNeighbor.boundaryPosECEF = seamECEF;
        }

        for (const auto& piece : segPieces) {
            outChunkSegments[{ piece.face, static_cast<uint32_t>(piece.chunkX), static_cast<uint32_t>(piece.chunkZ) }].push_back(piece);
        }
    }

    std::cout << "RoadGenerator: Sliced " << inSegments.size() << " road corridors into "
              << nextSubsegId - 1 << " subsegments across "
              << outChunkSegments.size() << " Level 15 Quadsphere chunks." << std::endl;
}

