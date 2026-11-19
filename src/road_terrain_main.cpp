#include <iostream>
#include <fstream>
#include <string>
#include "Engine.h"
#include "Road/RoadTerrainViewer.h"

int main(int argc, char* argv[]) {
    Engine engine;

    // Default to Quadsphere Somerset pack (SPHERE.md), falling back to legacy packs
    std::string terrainPackPath = "jughandle/somerset.fquadspherepack";
    if (!std::ifstream(terrainPackPath).good()) {
        terrainPackPath = "jughandle/new_jersey_05m.fterrainpack";
    }
    if (!std::ifstream(terrainPackPath).good()) {
        terrainPackPath = "jughandle/somerset_mc_lerc.fterrainpack";
    }
    if (!std::ifstream(terrainPackPath).good()) {
        terrainPackPath = "jughandle/jughandle_lerc.fterrainpack";
    }
    if (argc > 1) {
        terrainPackPath = argv[1];
    }

    std::string roadGeoJsonPath = "jughandle/somerset_roads.geojson";
    if (!std::ifstream(roadGeoJsonPath).good()) {
        roadGeoJsonPath = "jughandle/new_jersey_roads.geojson";
    }
    if (!std::ifstream(roadGeoJsonPath).good()) {
        roadGeoJsonPath = "jughandle/export.geojson";
    }
    if (argc > 2) {
        roadGeoJsonPath = argv[2];
    }

    std::cout << "Starting Road On Terrain Integration Showcase:" << std::endl;
    std::cout << "  Terrain Pack: " << terrainPackPath << std::endl;
    std::cout << "  Road GeoJSON: " << roadGeoJsonPath << std::endl;

    // Launch in 1440x900
    if (engine.init(1440, 900, "ForceEngine - Roads On Terrain Viewer")) {
        RoadTerrainViewer viewer;
        if (!viewer.init(engine, terrainPackPath, roadGeoJsonPath)) {
            std::cerr << "Failed to initialize Road Terrain Viewer!" << std::endl;
            return 1;
        }

        // Hook viewer into engine loop
        engine.onEvent = [&](const SDL_Event& event) {
            viewer.handleEvent(event);
        };

        engine.onUpdate = [&](float dt) {
            viewer.update(dt);
        };

        engine.onRender = [&]() {
            viewer.render(engine);
        };

        engine.run();
    }

    return 0;
}

