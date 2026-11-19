#include <iostream>
#include <fstream>
#include <string>
#include "Road/RoadBaker.h"

int main(int argc, char* argv[]) {
    RoadBakeOptions options;
    MultiChunkBakeOptions multiOptions;
    bool isMultiChunk = false;

    // Check for default terrain pack availability (prefer Quadsphere LiDAR)
    std::string defaultTerrain = "";
    if (std::ifstream("jughandle/somerset.fquadspherepack").good()) {
        defaultTerrain = "jughandle/somerset.fquadspherepack";
    } else if (std::ifstream("jughandle/new_jersey.fquadspherepack").good()) {
        defaultTerrain = "jughandle/new_jersey.fquadspherepack";
    } else if (std::ifstream("jughandle/new_jersey_05m.fterrainpack").good()) {
        defaultTerrain = "jughandle/new_jersey_05m.fterrainpack";
    } else if (std::ifstream("jughandle/somerset_mc_lerc.fterrainpack").good()) {
        defaultTerrain = "jughandle/somerset_mc_lerc.fterrainpack";
    } else if (std::ifstream("jughandle/jughandle_lerc.fterrainpack").good()) {
        defaultTerrain = "jughandle/jughandle_lerc.fterrainpack";
    }
    options.terrainPackPath = defaultTerrain;
    multiOptions.terrainPackPath = defaultTerrain;

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "ForceEngine Road Baker CLI\n"
                      << "Usage: RoadBaker [options]\n\n"
                      << "Single Chunk Options:\n"
                      << "  -i, --input <geojson>    Input vector road GeoJSON (default: jughandle/export.geojson)\n"
                      << "  -o, --output <froad>     Output baked .froad binary (default: assets/maps/jughandle_500m.froad)\n"
                      << "  -t, --terrain <fterrain> Terrain container for draping (default: somerset_mc_lerc.fterrainpack)\n"
                      << "  --no-terrain             Disable terrain draping (bake on flat plane)\n"
                      << "  --all                    Bake all road features across full bounds (disable 500m filter)\n"
                      << "  --ground-offset <m>      Offset above terrain for ground roads (default: 0.15m)\n"
                      << "  --bridge-clearance <m>   Elevation clearance for overpass bridges (default: 5.5m)\n\n"
                      << "Multi-Chunk Domain Decomposition Options:\n"
                      << "  --multi-chunk            Enable multi-chunk 512x512m domain decomposition\n"
                      << "  --chunks-dir <dir>       Directory to output baked chunk .froad files (default: assets/maps/new_jersey_chunks)\n"
                      << "  --chunk-size <m>         Chunk grid size in meters (default: 512.0m)\n"
                      << "  --halo-margin <m>        Boundary halo margin for smooth solving (default: 96.0m)\n"
                      << "  -h, --help               Show this help message\n";
            return 0;
        } else if (arg == "--multi-chunk") {
            isMultiChunk = true;
        } else if (arg == "--chunks-dir" && i + 1 < argc) {
            isMultiChunk = true;
            multiOptions.outputDir = argv[++i];
        } else if (arg == "--chunk-size" && i + 1 < argc) {
            multiOptions.chunkSize = std::stof(argv[++i]);
        } else if (arg == "--halo-margin" && i + 1 < argc) {
            multiOptions.haloMargin = std::stof(argv[++i]);
        } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
            options.geoJsonPath = argv[++i];
            multiOptions.geoJsonPath = options.geoJsonPath;
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            options.outputPath = argv[++i];
        } else if ((arg == "-t" || arg == "--terrain") && i + 1 < argc) {
            options.terrainPackPath = argv[++i];
            multiOptions.terrainPackPath = options.terrainPackPath;
        } else if (arg == "--no-terrain") {
            options.terrainPackPath = "";
            options.drapeOnTerrain = false;
            multiOptions.terrainPackPath = "";
        } else if (arg == "--all") {
            options.filter500mOnly = false;
        } else if (arg == "--ground-offset" && i + 1 < argc) {
            float val = std::stof(argv[++i]);
            options.groundOffset = val;
            multiOptions.groundOffset = val;
        } else if (arg == "--bridge-clearance" && i + 1 < argc) {
            float val = std::stof(argv[++i]);
            options.bridgeClearance = val;
            multiOptions.bridgeClearance = val;
        } else if (i == 1 && arg[0] != '-') {
            // Positional shorthand: RoadBaker <geojson> [output] [terrain]
            options.geoJsonPath = arg;
            multiOptions.geoJsonPath = arg;
        } else if (i == 2 && arg[0] != '-') {
            options.outputPath = arg;
        } else if (i == 3 && arg[0] != '-') {
            options.terrainPackPath = arg;
            multiOptions.terrainPackPath = arg;
        }
    }

    if (isMultiChunk) {
        auto result = RoadBaker::bakeChunks(multiOptions);
        if (!result.success) {
            std::cerr << "RoadBaker Multi-Chunk FAILED: " << result.errorMessage << std::endl;
            return 1;
        }
    } else {
        auto result = RoadBaker::bake(options);
        if (!result.success) {
            std::cerr << "RoadBaker FAILED: " << result.errorMessage << std::endl;
            return 1;
        }
    }

    return 0;
}

