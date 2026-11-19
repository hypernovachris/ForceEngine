#ifndef ROAD_ELEVATION_SOLVER_H
#define ROAD_ELEVATION_SOLVER_H

#include <vector>
#include <functional>
#include <glm/glm/glm.hpp>
#include "Road/RoadTypes.h"

struct PinnedBoundaryNode {
    glm::vec2 pos2D{0.0f};
    float elevation = 0.0f;
};

struct RoadElevationSolverOptions {
    float k_tension = 100.0f;       // Stiffness for tension / curvature minimization (elastic beam)
    float k_adhesion = 10.0f;       // Stiffness for adhering to terrain
    float k_repulsion = 200.0f;     // Stiffness for directional overpass clearance repulsion
    float minClearance = 5.5f;      // Minimum vertical clearance between crossing layers (meters)
    float groundOffset = 0.15f;     // Height above ground terrain (meters)
    float maxSegmentLength = 12.0f; // Subdivide control segments longer than this (meters)
    int maxSweeps = 200;            // Number of Gauss-Seidel relaxation sweeps
    float omega = 1.25f;            // Successive Over-Relaxation (SOR) factor for unconstrained nodes
    float tolerance = 0.001f;       // Sub-millimeter convergence threshold (1mm)
    bool verbose = false;

    // Dirichlet boundary conditions for multi-chunk domain decomposition
    std::vector<PinnedBoundaryNode> pinnedNodes;
};

class RoadElevationSolver {
public:
    // Solve elevation for all road segments given a terrain elevation sampling function
    static void solve(std::vector<ProcessedRoadSegment>& segments,
                      const std::function<float(float x, float z)>& sampleTerrainElevation,
                      const RoadElevationSolverOptions& options = RoadElevationSolverOptions());

    // Solve vertical elevation (h in meters above Earth sphere) for Quadsphere segments
    static void solveQuadsphere(std::vector<ProcessedRoadSegment>& segments,
                                const std::function<float(double latDeg, double lonDeg)>& sampleTerrainElevation,
                                const RoadElevationSolverOptions& options = RoadElevationSolverOptions());
};

#endif // ROAD_ELEVATION_SOLVER_H

