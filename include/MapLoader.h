#ifndef MAP_LOADER_H
#define MAP_LOADER_H

#include <string>
#include <memory>
#include <glm/glm/glm.hpp>
#include "MapBinary.h"

class Engine;

struct MapLoadResult {
    bool success = false;
    glm::vec3 playerStart{0.0f};
    float playerStartYaw = 0.0f;
    float playerStartPitch = 0.0f;
    uint32_t meshCount = 0;
    uint32_t colliderCount = 0;
    uint32_t triggerCount = 0;
    uint32_t lightCount = 0;
};

class MapLoader {
public:
    static MapLoadResult loadMap(const std::string& filepath, Engine& engine);
};

#endif // MAP_LOADER_H
