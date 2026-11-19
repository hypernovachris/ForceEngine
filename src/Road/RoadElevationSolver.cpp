#include "Road/RoadElevationSolver.h"
#include "Road/RoadGenerator.h"
#include "Terrain/QuadsphereMath.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace {

// Compute closest points between two 2D line segments [p1, q1] and [p2, q2]
void closestPtSegmentSegment2D(const glm::vec2& p1, const glm::vec2& q1,
                               const glm::vec2& p2, const glm::vec2& q2,
                               float& s, float& t,
                               glm::vec2& c1, glm::vec2& c2) {
    glm::vec2 d1 = q1 - p1;
    glm::vec2 d2 = q2 - p2;
    glm::vec2 r = p1 - p2;
    float a = glm::dot(d1, d1);
    float e = glm::dot(d2, d2);
    float f = glm::dot(d2, r);

    if (a <= 1e-6f && e <= 1e-6f) {
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return;
    }
    if (a <= 1e-6f) {
        s = 0.0f;
        t = std::clamp(f / e, 0.0f, 1.0f);
    } else {
        float c = glm::dot(d1, r);
        if (e <= 1e-6f) {
            t = 0.0f;
            s = std::clamp(-c / a, 0.0f, 1.0f);
        } else {
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b;
            if (denom > 1e-6f) {
                s = std::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            } else {
                s = 0.0f;
            }
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = std::clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = std::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

struct SolverNode {
    glm::vec2 pos2D{0.0f};
    float y = 0.0f;
    float targetY = 0.0f;
    bool hasAdhesion = false;
    bool isTunnel = false;
    bool isPinned = false; // Locked Dirichlet boundary node (multi-chunk domain decomposition)
    int effLayer = 0;
    std::vector<std::pair<int, float>> neighbors; // (nodeIdx, deltaS)
};

struct EndpointCluster {
    int nodeIdx = -1;
    glm::vec2 pos{0.0f};
};

struct RepulsionConstraint {
    int u0 = -1, u1 = -1; // Upper segment nodes
    int v0 = -1, v1 = -1; // Lower segment nodes
    float t = 0.0f;       // Barycentric weight on upper edge
    float u = 0.0f;       // Barycentric weight on lower edge
    float minClearance = 5.5f;
    float stiffness = 200.0f;
};

inline int getEffectiveLayer(const ProcessedRoadSegment& seg) {
    int eff = seg.layer;
    if (eff == 0 && seg.isBridge) eff = 1;
    return eff;
}

} // namespace

void RoadElevationSolver::solve(std::vector<ProcessedRoadSegment>& segments,
                                const std::function<float(float x, float z)>& sampleTerrainElevation,
                                const RoadElevationSolverOptions& options) {
    if (segments.empty()) return;

    auto tStart = std::chrono::high_resolution_clock::now();

    // ------------------------------------------------------------------------
    // Step 2: Build Solver Graph & Couple Shared Junction Nodes
    // ------------------------------------------------------------------------
    std::vector<SolverNode> solverNodes;
    std::vector<EndpointCluster> endpointClusters;
    const float mergeDist = 0.35f; // Cluster endpoints within 35cm

    // Map: [segmentIdx][pointIdx] -> solverNodeIdx
    std::vector<std::vector<int>> nodeIndices(segments.size());

    for (size_t s = 0; s < segments.size(); ++s) {
        const auto& seg = segments[s];
        nodeIndices[s].resize(seg.controlPoints.size(), -1);
        int effLayer = getEffectiveLayer(seg);
        bool segIsGround = (!seg.isBridge && seg.layer == 0 && !seg.isTunnel);
        bool segIsTunnel = seg.isTunnel;

        for (size_t k = 0; k < seg.controlPoints.size(); ++k) {
            const auto& pt = seg.controlPoints[k];
            glm::vec2 p2D(pt.x, pt.z);

            bool isEndpoint = (k == 0 || k == seg.controlPoints.size() - 1);

            int nodeIdx = -1;
            if (isEndpoint) {
                // Check if an existing endpoint cluster matches
                for (const auto& cl : endpointClusters) {
                    if (glm::distance(cl.pos, p2D) < mergeDist) {
                        nodeIdx = cl.nodeIdx;
                        break;
                    }
                }

                if (nodeIdx >= 0) {
                    // Shared junction node: update attributes
                    if (segIsGround) {
                        solverNodes[nodeIdx].hasAdhesion = true;
                    }
                    if (segIsTunnel) {
                        solverNodes[nodeIdx].isTunnel = true;
                    }
                    solverNodes[nodeIdx].effLayer = std::max(solverNodes[nodeIdx].effLayer, effLayer);
                } else {
                    // New endpoint node
                    nodeIdx = (int)solverNodes.size();
                    SolverNode sn;
                    sn.pos2D = p2D;
                    sn.y = pt.y;
                    sn.hasAdhesion = segIsGround;
                    sn.isTunnel = segIsTunnel;
                    sn.effLayer = effLayer;
                    solverNodes.push_back(sn);

                    endpointClusters.push_back({ nodeIdx, p2D });
                }
            } else {
                // Internal control point: unique to this segment
                nodeIdx = (int)solverNodes.size();
                SolverNode sn;
                sn.pos2D = p2D;
                sn.y = pt.y;
                sn.hasAdhesion = segIsGround;
                sn.isTunnel = segIsTunnel;
                sn.effLayer = effLayer;
                solverNodes.push_back(sn);
            }

            nodeIndices[s][k] = nodeIdx;
        }
    }

    // ------------------------------------------------------------------------
    // Step 3: Populate Incident Graph Edges & Arc-Length Spacing (deltaS)
    // ------------------------------------------------------------------------
    for (size_t s = 0; s < segments.size(); ++s) {
        for (size_t k = 0; k + 1 < nodeIndices[s].size(); ++k) {
            int u = nodeIndices[s][k];
            int v = nodeIndices[s][k + 1];
            if (u == v) continue;

            float ds = std::max(0.1f, glm::distance(solverNodes[u].pos2D, solverNodes[v].pos2D));

            bool hasV = false;
            for (const auto& nb : solverNodes[u].neighbors) {
                if (nb.first == v) { hasV = true; break; }
            }
            if (!hasV) solverNodes[u].neighbors.push_back({ v, ds });

            bool hasU = false;
            for (const auto& nb : solverNodes[v].neighbors) {
                if (nb.first == u) { hasU = true; break; }
            }
            if (!hasU) solverNodes[v].neighbors.push_back({ u, ds });
        }
    }

    // ------------------------------------------------------------------------
    // Step 4: Initialize Terrain Elevations & Adhesion Targets
    // ------------------------------------------------------------------------
    for (size_t i = 0; i < solverNodes.size(); ++i) {
        auto& sn = solverNodes[i];
        float groundY = sampleTerrainElevation ? sampleTerrainElevation(sn.pos2D.x, sn.pos2D.y) : 0.0f;
        sn.targetY = groundY + options.groundOffset;

        if (sn.hasAdhesion) {
            sn.y = sn.targetY;
        } else {
            float elevOffset = (float)std::max(1, sn.effLayer) * options.minClearance;
            sn.y = std::max(sn.y, sn.targetY + elevOffset);
        }
    }

    // Apply Dirichlet boundary conditions (lock pinned boundary nodes to matching adjacent chunk elevation)
    for (const auto& pin : options.pinnedNodes) {
        for (auto& sn : solverNodes) {
            if (glm::distance(sn.pos2D, pin.pos2D) < mergeDist) {
                sn.isPinned = true;
                sn.y = pin.elevation;
            }
        }
    }

    // ------------------------------------------------------------------------
    // Step 5: Precompute Directional Repulsion Constraints in 2D (XZ Plane)
    // ------------------------------------------------------------------------
    std::vector<RepulsionConstraint> repConstraints;

    // Segment AABB acceleration
    struct SegAABB {
        float minX, maxX, minZ, maxZ;
    };
    std::vector<SegAABB> segAABBs(segments.size());
    for (size_t s = 0; s < segments.size(); ++s) {
        float minX = 1e9f, maxX = -1e9f, minZ = 1e9f, maxZ = -1e9f;
        for (int nodeIdx : nodeIndices[s]) {
            const auto& p = solverNodes[nodeIdx].pos2D;
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minZ = std::min(minZ, p.y);
            maxZ = std::max(maxZ, p.y);
        }
        segAABBs[s] = { minX, maxX, minZ, maxZ };
    }

    for (size_t a = 0; a < segments.size(); ++a) {
        int effA = getEffectiveLayer(segments[a]);

        for (size_t b = 0; b < segments.size(); ++b) {
            int effB = getEffectiveLayer(segments[b]);

            // Directional signed inequality: only upper layers repel lower layers
            if (effA <= effB) continue;

            // Connected segments (e.g. on-ramp merging into highway, approach connecting to bridge)
            // share a junction and must NEVER repel each other!
            bool sharesJunction = false;
            for (int na : nodeIndices[a]) {
                for (int nb : nodeIndices[b]) {
                    if (na == nb) {
                        sharesJunction = true;
                        break;
                    }
                }
                if (sharesJunction) break;
            }
            if (sharesJunction) continue;

            float dThresh = 0.5f * (segments[a].totalWidth + segments[b].totalWidth);

            // Bounding box test
            if (segAABBs[a].minX - dThresh > segAABBs[b].maxX ||
                segAABBs[a].maxX + dThresh < segAABBs[b].minX ||
                segAABBs[a].minZ - dThresh > segAABBs[b].maxZ ||
                segAABBs[a].maxZ + dThresh < segAABBs[b].minZ) {
                continue;
            }

            // Test edge pairs
            for (size_t ka = 0; ka + 1 < nodeIndices[a].size(); ++ka) {
                int u0 = nodeIndices[a][ka];
                int u1 = nodeIndices[a][ka + 1];

                for (size_t kb = 0; kb + 1 < nodeIndices[b].size(); ++kb) {
                    int v0 = nodeIndices[b][kb];
                    int v1 = nodeIndices[b][kb + 1];

                    // Ignore connected edges sharing a node
                    if (u0 == v0 || u0 == v1 || u1 == v0 || u1 == v1) continue;

                    // Angle check: skip parallel or near-parallel segments (they run side-by-side, not crossing)
                    glm::vec2 dirA = solverNodes[u1].pos2D - solverNodes[u0].pos2D;
                    glm::vec2 dirB = solverNodes[v1].pos2D - solverNodes[v0].pos2D;
                    float lenA = glm::length(dirA);
                    float lenB = glm::length(dirB);
                    if (lenA < 0.01f || lenB < 0.01f) continue;
                    float cross2D = std::abs(dirA.x * dirB.y - dirA.y * dirB.x);
                    float sinTheta = cross2D / (lenA * lenB);
                    if (sinTheta < 0.15f) continue; // Parallel/shallow angle: do not repel

                    // Edge AABB test
                    float edgeMinAx = std::min(solverNodes[u0].pos2D.x, solverNodes[u1].pos2D.x) - dThresh;
                    float edgeMaxAx = std::max(solverNodes[u0].pos2D.x, solverNodes[u1].pos2D.x) + dThresh;
                    float edgeMinAz = std::min(solverNodes[u0].pos2D.y, solverNodes[u1].pos2D.y) - dThresh;
                    float edgeMaxAz = std::max(solverNodes[u0].pos2D.y, solverNodes[u1].pos2D.y) + dThresh;

                    float edgeMinBx = std::min(solverNodes[v0].pos2D.x, solverNodes[v1].pos2D.x);
                    float edgeMaxBx = std::max(solverNodes[v0].pos2D.x, solverNodes[v1].pos2D.x);
                    float edgeMinBz = std::min(solverNodes[v0].pos2D.y, solverNodes[v1].pos2D.y);
                    float edgeMaxBz = std::max(solverNodes[v0].pos2D.y, solverNodes[v1].pos2D.y);

                    if (edgeMinAx > edgeMaxBx || edgeMaxAx < edgeMinBx ||
                        edgeMinAz > edgeMaxBz || edgeMaxAz < edgeMinBz) {
                        continue;
                    }

                    float sParam = 0.0f, tParam = 0.0f;
                    glm::vec2 cA{0.0f}, cB{0.0f};
                    closestPtSegmentSegment2D(solverNodes[u0].pos2D, solverNodes[u1].pos2D,
                                             solverNodes[v0].pos2D, solverNodes[v1].pos2D,
                                             sParam, tParam, cA, cB);

                    float d2D = glm::distance(cA, cB);
                    if (d2D < dThresh) {
                        RepulsionConstraint rc;
                        rc.u0 = u0; rc.u1 = u1;
                        rc.v0 = v0; rc.v1 = v1;
                        rc.t = sParam;
                        rc.u = tParam;
                        rc.minClearance = options.minClearance;
                        rc.stiffness = options.k_repulsion;
                        repConstraints.push_back(rc);
                    }
                }
            }
        }
    }

    if (options.verbose) {
        std::cout << "RoadElevationSolver: Built graph with " << solverNodes.size()
                  << " nodes, " << endpointClusters.size() << " junction clusters, and "
                  << repConstraints.size() << " directional repulsion constraints." << std::endl;
    }

    // ------------------------------------------------------------------------
    // Step 6: Over-Damped Gauss-Seidel Relaxation with SOR
    // ------------------------------------------------------------------------
    std::vector<float> f_rep(solverNodes.size(), 0.0f);
    std::vector<float> k_rep(solverNodes.size(), 0.0f);
    std::vector<uint8_t> hasActiveRep(solverNodes.size(), 0);

    int convergedSweeps = options.maxSweeps;

    for (int sweep = 0; sweep < options.maxSweeps; ++sweep) {
        float maxDelta = 0.0f;

        // 1. Evaluate one-sided signed repulsion constraints
        std::fill(f_rep.begin(), f_rep.end(), 0.0f);
        std::fill(k_rep.begin(), k_rep.end(), 0.0f);
        std::fill(hasActiveRep.begin(), hasActiveRep.end(), 0);

        for (const auto& rc : repConstraints) {
            float yUpper = (1.0f - rc.t) * solverNodes[rc.u0].y + rc.t * solverNodes[rc.u1].y;
            float yLower = (1.0f - rc.u) * solverNodes[rc.v0].y + rc.u * solverNodes[rc.v1].y;

            // Maximum civil engineering clearance ceiling: upper road should never exceed local ground + layer * clearance + 1.5m
            float groundUpper = (1.0f - rc.t) * solverNodes[rc.u0].targetY + rc.t * solverNodes[rc.u1].targetY;
            float maxAllowedUpper = groundUpper + (float)std::max(1, solverNodes[rc.u0].effLayer) * rc.minClearance + 1.5f;
            float targetUpper = std::min(yLower + rc.minClearance, maxAllowedUpper);
            float deltaY = targetUpper - yUpper;

            if (deltaY > 0.0f) {
                float f = rc.stiffness * deltaY;
                float w0 = 1.0f - rc.t;
                float w1 = rc.t;

                f_rep[rc.u0] += w0 * f;
                k_rep[rc.u0] += w0 * rc.stiffness;
                hasActiveRep[rc.u0] = 1;

                f_rep[rc.u1] += w1 * f;
                k_rep[rc.u1] += w1 * rc.stiffness;
                hasActiveRep[rc.u1] = 1;

                // Push lower road down ONLY if not pinned by adhesion
                if (!solverNodes[rc.v0].hasAdhesion) {
                    float vw0 = 1.0f - rc.u;
                    f_rep[rc.v0] -= vw0 * f;
                    k_rep[rc.v0] += vw0 * rc.stiffness;
                    hasActiveRep[rc.v0] = 1;
                }
                if (!solverNodes[rc.v1].hasAdhesion) {
                    float vw1 = rc.u;
                    f_rep[rc.v1] -= vw1 * f;
                    k_rep[rc.v1] += vw1 * rc.stiffness;
                    hasActiveRep[rc.v1] = 1;
                }
            }
        }

        // 2. In-place Gauss-Seidel update per node
        for (size_t i = 0; i < solverNodes.size(); ++i) {
            auto& node = solverNodes[i];
            if (node.isPinned) {
                continue; // Dirichlet boundary condition: fixed height from adjacent chunk seam
            }

            float fTot = f_rep[i];
            float kTot = k_rep[i];

            // Tension force: weighted by inverse arc length (deltaS)
            // Scaling by maxSegmentLength / Li ensures consistent physical beam stiffness across resolutions
            // so increasing k_tension smoothly lifts approach segments before/after overpasses
            if (!node.neighbors.empty()) {
                float sumDeltaS = 0.0f;
                for (const auto& nb : node.neighbors) sumDeltaS += nb.second;
                float Li = sumDeltaS / (float)node.neighbors.size();

                float sumTensionNum = 0.0f;
                float sumTensionDenom = 0.0f;
                for (const auto& nb : node.neighbors) {
                    sumTensionNum += (solverNodes[nb.first].y - node.y) / nb.second;
                    sumTensionDenom += 1.0f / nb.second;
                }
                float tensionScale = options.k_tension * (options.maxSegmentLength / Li);
                fTot += tensionScale * sumTensionNum;
                kTot += tensionScale * sumTensionDenom;
            }

            // Adhesion / Terrain Floor barrier constraint:
            // - Ground roads (hasAdhesion=true): strongly repelled upward if below terrain, and gently pulled down if above.
            // - Overpasses, bridges, and junctions: respond to upward adhesion repulsion if below terrain, but feel 0 downward pull when elevated.
            bool isUnderground = false;
            if (!node.isTunnel) {
                float delta = node.targetY - node.y;
                if (delta > 0.0f) {
                    // Road/overpass/junction is below terrain level: repel upward sharply
                    float kEff = options.k_adhesion * (1.0f + 4.0f * delta);
                    fTot += kEff * delta;
                    kTot += kEff;
                    isUnderground = (delta > 0.05f);
                } else if (node.hasAdhesion) {
                    // Only surface roads feel downward pull towards terrain;
                    // overpasses and elevated junctions feel 0 downward pull (free to stay elevated)
                    float kEff = options.k_adhesion * 0.15f;
                    fTot += kEff * delta;
                    kTot += kEff;
                }
            }

            if (kTot > 1e-6f) {
                // Over-relaxation: use omega=1.0 when active inequality or steep barrier constraints
                // are present to prevent chattering, and options.omega for smooth linear nodes
                float localOmega = (hasActiveRep[i] || isUnderground) ? 1.0f : options.omega;
                float dy = localOmega * (fTot / kTot);
                node.y += dy;
                maxDelta = std::max(maxDelta, std::abs(dy));
            }
        }

        if (maxDelta < options.tolerance) {
            convergedSweeps = sweep + 1;
            break;
        }
    }

    auto tEnd = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(tEnd - tStart).count();

    if (options.verbose) {
        std::cout << "RoadElevationSolver: Converged in " << convergedSweeps
                  << " sweeps (" << ms << " ms)" << std::endl;
    }

    // ------------------------------------------------------------------------
    // Step 7: Write Solved Elevations Back to Control Points & Regenerate Splines
    // ------------------------------------------------------------------------
    for (size_t s = 0; s < segments.size(); ++s) {
        auto& seg = segments[s];
        for (size_t k = 0; k < seg.controlPoints.size(); ++k) {
            int nodeIdx = nodeIndices[s][k];
            seg.controlPoints[k].x = solverNodes[nodeIdx].pos2D.x;
            seg.controlPoints[k].y = solverNodes[nodeIdx].y;
            seg.controlPoints[k].z = solverNodes[nodeIdx].pos2D.y;
        }

        // Regenerate uniform centripetal Catmull-Rom spline points through solved control points
        seg.splinePoints = RoadGenerator::generateSpline(seg.controlPoints, seg.sampleStep);
        if (!seg.splinePoints.empty()) {
            seg.length = seg.splinePoints.back().distance;
        }
    }
}

void RoadElevationSolver::solveQuadsphere(std::vector<ProcessedRoadSegment>& segments,
                                          const std::function<float(double latDeg, double lonDeg)>& sampleTerrainElevation,
                                          const RoadElevationSolverOptions& options) {
    if (segments.empty()) return;

    auto tStart = std::chrono::high_resolution_clock::now();

    struct QuadSolverNode {
        glm::vec2 pos2D{0.0f};
        glm::dvec3 n_up{0.0, 1.0, 0.0};
        double lat = 0.0, lon = 0.0;
        float h = 0.0f;
        float targetH = 0.0f;
        bool hasAdhesion = false;
        bool isTunnel = false;
        int effLayer = 0;
        bool isPinned = false;
        std::vector<std::pair<int, float>> neighbors;
    };

    std::vector<QuadSolverNode> solverNodes;
    std::vector<EndpointCluster> endpointClusters;
    const float mergeDist = 0.35f;

    std::vector<std::vector<int>> nodeIndices(segments.size());

    for (size_t s = 0; s < segments.size(); ++s) {
        const auto& seg = segments[s];
        nodeIndices[s].resize(seg.controlPoints.size(), -1);
        int effLayer = getEffectiveLayer(seg);
        bool segIsGround = (!seg.isBridge && seg.layer == 0 && !seg.isTunnel);
        bool segIsTunnel = seg.isTunnel;

        for (size_t k = 0; k < seg.controlPoints.size(); ++k) {
            const auto& pt = seg.controlPoints[k];
            glm::vec2 p2D(pt.x, pt.z);

            glm::dvec3 ecef = seg.chunkCenterECEF + glm::dvec3(pt);
            double r = glm::length(ecef);
            glm::dvec3 n_up = (r > 1e-3) ? (ecef / r) : glm::dvec3(0.0, 1.0, 0.0);
            float currentH = static_cast<float>(r - Quadsphere::EARTH_RADIUS);
            double lat = 0.0, lon = 0.0, dummyH = 0.0;
            Quadsphere::ecefToGeodetic(ecef, lat, lon, dummyH);

            bool isEndpoint = (k == 0 || k == seg.controlPoints.size() - 1);
            int nodeIdx = -1;

            if (isEndpoint) {
                for (const auto& cl : endpointClusters) {
                    if (glm::distance(cl.pos, p2D) < mergeDist) {
                        nodeIdx = cl.nodeIdx;
                        break;
                    }
                }

                if (nodeIdx >= 0) {
                    if (segIsGround) solverNodes[nodeIdx].hasAdhesion = true;
                    if (segIsTunnel) solverNodes[nodeIdx].isTunnel = true;
                    solverNodes[nodeIdx].effLayer = std::max(solverNodes[nodeIdx].effLayer, effLayer);
                } else {
                    nodeIdx = (int)solverNodes.size();
                    QuadSolverNode sn;
                    sn.pos2D = p2D;
                    sn.n_up = n_up;
                    sn.lat = lat;
                    sn.lon = lon;
                    sn.h = currentH;
                    sn.hasAdhesion = segIsGround;
                    sn.isTunnel = segIsTunnel;
                    sn.effLayer = effLayer;
                    solverNodes.push_back(sn);
                    endpointClusters.push_back({ nodeIdx, p2D });
                }
            } else {
                nodeIdx = (int)solverNodes.size();
                QuadSolverNode sn;
                sn.pos2D = p2D;
                sn.n_up = n_up;
                sn.lat = lat;
                sn.lon = lon;
                sn.h = currentH;
                sn.hasAdhesion = segIsGround;
                sn.isTunnel = segIsTunnel;
                sn.effLayer = effLayer;
                solverNodes.push_back(sn);
            }

            nodeIndices[s][k] = nodeIdx;
        }
    }

    // Step 3: Populate Incident Graph Edges & Arc-Length Spacing
    for (size_t s = 0; s < segments.size(); ++s) {
        for (size_t k = 0; k + 1 < nodeIndices[s].size(); ++k) {
            int u = nodeIndices[s][k];
            int v = nodeIndices[s][k + 1];
            if (u == v) continue;

            float ds = std::max(0.1f, glm::distance(solverNodes[u].pos2D, solverNodes[v].pos2D));

            bool hasV = false;
            for (const auto& nb : solverNodes[u].neighbors) {
                if (nb.first == v) { hasV = true; break; }
            }
            if (!hasV) solverNodes[u].neighbors.push_back({ v, ds });

            bool hasU = false;
            for (const auto& nb : solverNodes[v].neighbors) {
                if (nb.first == u) { hasU = true; break; }
            }
            if (!hasU) solverNodes[v].neighbors.push_back({ u, ds });
        }
    }

    // Step 4: Initialize Terrain Elevations & Adhesion Targets
    for (size_t i = 0; i < solverNodes.size(); ++i) {
        auto& sn = solverNodes[i];
        float groundH = sampleTerrainElevation ? sampleTerrainElevation(sn.lat, sn.lon) : 0.0f;
        sn.targetH = groundH + options.groundOffset;

        if (sn.hasAdhesion) {
            sn.h = sn.targetH;
        } else {
            float elevOffset = (float)std::max(1, sn.effLayer) * options.minClearance;
            sn.h = std::max(sn.h, sn.targetH + elevOffset);
        }
    }

    // Apply Dirichlet boundary conditions (lock pinned boundary nodes)
    for (const auto& pin : options.pinnedNodes) {
        for (auto& sn : solverNodes) {
            if (glm::distance(sn.pos2D, pin.pos2D) < mergeDist) {
                sn.isPinned = true;
                sn.h = pin.elevation;
            }
        }
    }

    // Step 5: Repulsion Constraints in 2D
    std::vector<RepulsionConstraint> repConstraints;
    struct SegAABB {
        float minX, maxX, minZ, maxZ;
    };
    std::vector<SegAABB> segAABBs(segments.size());
    for (size_t s = 0; s < segments.size(); ++s) {
        float minX = 1e9f, maxX = -1e9f, minZ = 1e9f, maxZ = -1e9f;
        for (int nodeIdx : nodeIndices[s]) {
            const auto& p = solverNodes[nodeIdx].pos2D;
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minZ = std::min(minZ, p.y);
            maxZ = std::max(maxZ, p.y);
        }
        segAABBs[s] = { minX, maxX, minZ, maxZ };
    }

    for (size_t a = 0; a < segments.size(); ++a) {
        int effA = getEffectiveLayer(segments[a]);
        for (size_t b = 0; b < segments.size(); ++b) {
            int effB = getEffectiveLayer(segments[b]);
            if (effA <= effB) continue;

            bool sharesJunction = false;
            for (int na : nodeIndices[a]) {
                for (int nb : nodeIndices[b]) {
                    if (na == nb) { sharesJunction = true; break; }
                }
                if (sharesJunction) break;
            }
            if (sharesJunction) continue;

            float dThresh = 0.5f * (segments[a].totalWidth + segments[b].totalWidth);

            if (segAABBs[a].minX - dThresh > segAABBs[b].maxX ||
                segAABBs[a].maxX + dThresh < segAABBs[b].minX ||
                segAABBs[a].minZ - dThresh > segAABBs[b].maxZ ||
                segAABBs[a].maxZ + dThresh < segAABBs[b].minZ) {
                continue;
            }

            for (size_t ka = 0; ka + 1 < nodeIndices[a].size(); ++ka) {
                int u0 = nodeIndices[a][ka];
                int u1 = nodeIndices[a][ka + 1];

                for (size_t kb = 0; kb + 1 < nodeIndices[b].size(); ++kb) {
                    int v0 = nodeIndices[b][kb];
                    int v1 = nodeIndices[b][kb + 1];

                    if (u0 == v0 || u0 == v1 || u1 == v0 || u1 == v1) continue;

                    glm::vec2 dirA = solverNodes[u1].pos2D - solverNodes[u0].pos2D;
                    glm::vec2 dirB = solverNodes[v1].pos2D - solverNodes[v0].pos2D;
                    float lenA = glm::length(dirA);
                    float lenB = glm::length(dirB);
                    if (lenA < 0.01f || lenB < 0.01f) continue;
                    float cross2D = std::abs(dirA.x * dirB.y - dirA.y * dirB.x);
                    float sinTheta = cross2D / (lenA * lenB);
                    if (sinTheta < 0.15f) continue;

                    float sParam = 0.0f, tParam = 0.0f;
                    glm::vec2 cA{0.0f}, cB{0.0f};
                    closestPtSegmentSegment2D(solverNodes[u0].pos2D, solverNodes[u1].pos2D,
                                             solverNodes[v0].pos2D, solverNodes[v1].pos2D,
                                             sParam, tParam, cA, cB);

                    float d2D = glm::distance(cA, cB);
                    if (d2D < dThresh) {
                        RepulsionConstraint rc;
                        rc.u0 = u0; rc.u1 = u1;
                        rc.v0 = v0; rc.v1 = v1;
                        rc.t = sParam;
                        rc.u = tParam;
                        rc.minClearance = options.minClearance;
                        rc.stiffness = options.k_repulsion;
                        repConstraints.push_back(rc);
                    }
                }
            }
        }
    }

    // Step 6: Gauss-Seidel Successive Over-Relaxation (SOR) Iteration Loop
    int convergedSweeps = options.maxSweeps;
    for (int sweep = 0; sweep < options.maxSweeps; ++sweep) {
        float maxDelta = 0.0f;

        std::vector<float> f_rep(solverNodes.size(), 0.0f);
        std::vector<float> k_rep(solverNodes.size(), 0.0f);

        for (const auto& rc : repConstraints) {
            float yUpper = (1.0f - rc.t) * solverNodes[rc.u0].h + rc.t * solverNodes[rc.u1].h;
            float yLower = (1.0f - rc.u) * solverNodes[rc.v0].h + rc.u * solverNodes[rc.v1].h;
            float clearance = yUpper - yLower;

            if (clearance < rc.minClearance) {
                float penetration = rc.minClearance - clearance;
                float f = rc.stiffness * penetration;

                float w0 = 1.0f - rc.t;
                float w1 = rc.t;
                f_rep[rc.u0] += w0 * f;
                k_rep[rc.u0] += w0 * rc.stiffness;
                f_rep[rc.u1] += w1 * f;
                k_rep[rc.u1] += w1 * rc.stiffness;

                if (!solverNodes[rc.v0].hasAdhesion) {
                    float vw0 = 1.0f - rc.u;
                    f_rep[rc.v0] -= vw0 * f;
                    k_rep[rc.v0] += vw0 * rc.stiffness;
                }
                if (!solverNodes[rc.v1].hasAdhesion) {
                    float vw1 = rc.u;
                    f_rep[rc.v1] -= vw1 * f;
                    k_rep[rc.v1] += vw1 * rc.stiffness;
                }
            }
        }

        for (size_t i = 0; i < solverNodes.size(); ++i) {
            auto& node = solverNodes[i];
            if (node.isPinned) continue;

            float fTot = f_rep[i];
            float kTot = k_rep[i];

            if (!node.neighbors.empty()) {
                float sumDeltaS = 0.0f;
                for (const auto& nb : node.neighbors) sumDeltaS += nb.second;
                float Li = sumDeltaS / (float)node.neighbors.size();
                float scale = std::clamp(options.maxSegmentLength / Li, 0.5f, 2.5f);
                float k_eff = options.k_tension * scale;

                float sumW = 0.0f;
                float sumWY = 0.0f;
                for (const auto& nb : node.neighbors) {
                    float w = 1.0f / nb.second;
                    sumW += w;
                    sumWY += w * solverNodes[nb.first].h;
                }
                if (sumW > 1e-4f) {
                    fTot += k_eff * (sumWY - sumW * node.h);
                    kTot += k_eff * sumW;
                }
            }

            if (node.hasAdhesion) {
                fTot += options.k_adhesion * (node.targetH - node.h);
                kTot += options.k_adhesion;
            }

            if (kTot > 1e-4f) {
                float delta = fTot / kTot;
                float sor = node.hasAdhesion ? 1.0f : options.omega;
                float newH = node.h + sor * delta;
                if (node.hasAdhesion && !node.isTunnel) {
                    newH = std::max(newH, node.targetH);
                }
                float change = std::abs(newH - node.h);
                maxDelta = std::max(maxDelta, change);
                node.h = newH;
            }
        }

        if (maxDelta < options.tolerance) {
            convergedSweeps = sweep + 1;
            break;
        }
    }

    auto tEnd = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(tEnd - tStart).count();

    if (options.verbose) {
        std::cout << "RoadElevationSolver (Quadsphere): Converged in " << convergedSweeps
                  << " sweeps (" << ms << " ms)" << std::endl;
    }

    // Step 7: Write Solved Elevations Back to Control Points & Regenerate Splines
    for (size_t s = 0; s < segments.size(); ++s) {
        auto& seg = segments[s];
        for (size_t k = 0; k < seg.controlPoints.size(); ++k) {
            int nodeIdx = nodeIndices[s][k];
            float solvedH = solverNodes[nodeIdx].h;
            glm::dvec3 n_up = solverNodes[nodeIdx].n_up;
            glm::dvec3 posECEF = (Quadsphere::EARTH_RADIUS + solvedH) * n_up;
            seg.controlPoints[k] = glm::vec3(posECEF - seg.chunkCenterECEF);
            seg.elevation = solvedH;
        }

        // Regenerate uniform centripetal Catmull-Rom spline points through solved control points
        seg.splinePoints = RoadGenerator::generateSpline(seg.controlPoints, seg.sampleStep, &seg.chunkCenterECEF);
        if (!seg.splinePoints.empty()) {
            seg.length = seg.splinePoints.back().distance;
        }
    }
}

