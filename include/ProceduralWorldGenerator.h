#ifndef PROCEDURAL_WORLD_GENERATOR_H
#define PROCEDURAL_WORLD_GENERATOR_H

#include <memory>
#include <vector>
#include <glm/glm/glm.hpp>
#include "Entity.h"

struct WorldConfig {
    int gridSize = 25;       // 25x25 grid
    float cellSize = 2.0f;   // 2m per cell -> 50m x 50m world
    float wallHeight = 3.5f;
    int courtyardRadius = 4; // Central open courtyard (-4 to +4 cells)
    unsigned int seed = 1337;
};

class ProceduralWorldGenerator {
public:
    static void generate(std::shared_ptr<Entity> rootEntity, const WorldConfig& config = WorldConfig());
};

#endif // PROCEDURAL_WORLD_GENERATOR_H
