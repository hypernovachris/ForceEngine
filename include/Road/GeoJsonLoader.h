#ifndef GEOJSON_LOADER_H
#define GEOJSON_LOADER_H

#include <string>
#include <vector>
#include "Road/RoadTypes.h"

class GeoJsonLoader {
public:
    static bool load(const std::string& filepath,
                     std::vector<RawRoadFeature>& outFeatures,
                     double& outCenterLon, double& outCenterLat);

    // Ingest pure WGS84 GPS (lat, lon) features for Quadsphere spherical pipeline
    static bool loadGeodetic(const std::string& filepath,
                             std::vector<RawRoadFeature>& outFeatures);

    // Load GeoJSON and project GPS (lat, lon) into local ENU coordinates using a terrain pack's WGS84 ECEF anchor
    static bool loadWithAnchor(const std::string& filepath,
                               const double ecefAnchor[3],
                               const float enuRotation[9],
                               std::vector<RawRoadFeature>& outFeatures);
};

#endif // GEOJSON_LOADER_H

