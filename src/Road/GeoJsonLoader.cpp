#include "Road/GeoJsonLoader.h"
#include "Terrain/TerrainLoader.h"
#include <rapidjson/document.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

static float parseDimensionString(const std::string& str) {
    if (str.empty()) return -1.0f;
    try {
        size_t idx = 0;
        float val = std::stof(str, &idx);
        std::string unit = str.substr(idx);
        while (!unit.empty() && (unit[0] == ' ' || unit[0] == '\t')) unit.erase(0, 1);
        if (unit.find("ft") != std::string::npos || unit.find('\'') != std::string::npos) {
            val *= 0.3048f; // Feet to meters
        }
        return (val > 0.0f) ? val : -1.0f;
    } catch (...) {
        return -1.0f;
    }
}

bool GeoJsonLoader::load(const std::string& filepath,
                         std::vector<RawRoadFeature>& outFeatures,
                         double& outCenterLon, double& outCenterLat) {
    outFeatures.clear();

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "GeoJsonLoader: Failed to open " << filepath << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonStr = buffer.str();

    rapidjson::Document doc;
    doc.Parse(jsonStr.c_str());
    if (doc.HasParseError() || !doc.IsObject() || !doc.HasMember("features")) {
        std::cerr << "GeoJsonLoader: JSON parse error in " << filepath << std::endl;
        return false;
    }

    const auto& features = doc["features"];
    if (!features.IsArray()) return false;

    // Fixed geographic center for Somerset / US 202-206 Jughandle, unless center was provided
    if (std::abs(outCenterLon) < 1e-4 && std::abs(outCenterLat) < 1e-4) {
        outCenterLon = -74.623174;
        outCenterLat = 40.5831884;
    }

    const double degToRad = 3.14159265358979323846 / 180.0;
    double m_per_lat = 111139.0;
    double m_per_lon = 111139.0 * std::cos(outCenterLat * degToRad);

    uint32_t featureIdCounter = 1;

    for (rapidjson::SizeType i = 0; i < features.Size(); ++i) {
        const auto& feat = features[i];
        if (!feat.IsObject() || !feat.HasMember("geometry")) continue;

        const auto& geom = feat["geometry"];
        if (!geom.IsObject() || !geom.HasMember("type") || !geom.HasMember("coordinates")) continue;

        std::string geomType = geom["type"].GetString();
        if (geomType != "LineString") continue;

        const auto& coords = geom["coordinates"];
        if (!coords.IsArray() || coords.Size() < 2) continue;

        RawRoadFeature road;
        road.id = featureIdCounter++;

        if (feat.HasMember("id") && feat["id"].IsString()) {
            road.osmId = feat["id"].GetString();
        }

        if (feat.HasMember("properties") && feat["properties"].IsObject()) {
            const auto& props = feat["properties"];
            if (props.HasMember("name") && props["name"].IsString()) {
                road.name = props["name"].GetString();
            }
            if (props.HasMember("highway") && props["highway"].IsString()) {
                road.highway = props["highway"].GetString();
            }
            if (props.HasMember("ref") && props["ref"].IsString()) {
                road.ref = props["ref"].GetString();
            }
            if (props.HasMember("destination") && props["destination"].IsString()) {
                road.destination = props["destination"].GetString();
            }
            if (props.HasMember("lanes") && props["lanes"].IsString()) {
                road.lanes = std::max(1, std::atoi(props["lanes"].GetString()));
                road.hasExplicitLanes = true;
            } else {
                road.hasExplicitLanes = false;
                if (road.highway == "trunk") road.lanes = 2;
                else if (road.highway == "trunk_link") road.lanes = 1;
                else road.lanes = 2;
            }
            if (props.HasMember("oneway") && props["oneway"].IsString()) {
                road.oneway = (std::string(props["oneway"].GetString()) == "yes");
            } else {
                road.oneway = (road.highway == "trunk_link");
            }
            if (props.HasMember("width") && props["width"].IsString()) {
                road.parsedWidth = parseDimensionString(props["width"].GetString());
            }
            if (props.HasMember("lane_width") && props["lane_width"].IsString()) {
                road.parsedLaneWidth = parseDimensionString(props["lane_width"].GetString());
            } else if (props.HasMember("width:lanes") && props["width:lanes"].IsString()) {
                road.parsedLaneWidth = parseDimensionString(props["width:lanes"].GetString());
            }
            if (props.HasMember("shoulder") && props["shoulder"].IsString()) {
                road.shoulderTag = props["shoulder"].GetString();
            }
            if (props.HasMember("shoulder:width") && props["shoulder:width"].IsString()) {
                road.parsedShoulderWidth = parseDimensionString(props["shoulder:width"].GetString());
            } else if (props.HasMember("shoulder:width:right") && props["shoulder:width:right"].IsString()) {
                road.parsedShoulderWidth = parseDimensionString(props["shoulder:width"].GetString());
            }
            if (props.HasMember("layer")) {
                if (props["layer"].IsString()) road.layer = std::atoi(props["layer"].GetString());
                else if (props["layer"].IsInt()) road.layer = props["layer"].GetInt();
            }
            if (props.HasMember("bridge") && props["bridge"].IsString()) {
                road.isBridge = (std::string(props["bridge"].GetString()) == "yes");
            }
            if (props.HasMember("tunnel") && props["tunnel"].IsString()) {
                road.isTunnel = (std::string(props["tunnel"].GetString()) == "yes");
            }
        }

        // Base elevation: AASHTO standard 5.5m for layer 1 highway overpasses, -5.0m for tunnels
        if (road.isBridge || road.layer > 0) {
            road.elevation = (road.layer > 0 ? (float)road.layer : 1.0f) * 5.5f;
        } else if (road.isTunnel || road.layer < 0) {
            road.elevation = (road.layer < 0 ? (float)road.layer : -1.0f) * 5.0f;
        } else {
            road.elevation = 0.05f;
        }

        for (rapidjson::SizeType c = 0; c < coords.Size(); ++c) {
            const auto& pt = coords[c];
            if (pt.IsArray() && pt.Size() >= 2) {
                double lon = pt[0].GetDouble();
                double lat = pt[1].GetDouble();

                float x = (float)((lon - outCenterLon) * m_per_lon);
                float z = (float)(-(lat - outCenterLat) * m_per_lat);
                float y = road.elevation;

                road.rawPoints.push_back(glm::vec3(x, y, z));
                road.gpsPoints.push_back(glm::dvec2(lon, lat));
            }
        }

        if (road.rawPoints.size() >= 2) {
            outFeatures.push_back(road);
        }
    }

    std::cout << "GeoJsonLoader: Ingested " << outFeatures.size()
              << " LineString road features from " << filepath << std::endl;
    return true;
}

bool GeoJsonLoader::loadGeodetic(const std::string& filepath,
                                 std::vector<RawRoadFeature>& outFeatures) {
    double dummyLon = 0.0, dummyLat = 0.0;
    return load(filepath, outFeatures, dummyLon, dummyLat);
}

bool GeoJsonLoader::loadWithAnchor(const std::string& filepath,
                                   const double ecefAnchor[3],
                                   const float enuRotation[9],
                                   std::vector<RawRoadFeature>& outFeatures) {
    outFeatures.clear();

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "GeoJsonLoader: Failed to open " << filepath << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonStr = buffer.str();

    rapidjson::Document doc;
    doc.Parse(jsonStr.c_str());
    if (doc.HasParseError() || !doc.IsObject() || !doc.HasMember("features")) {
        std::cerr << "GeoJsonLoader: JSON parse error in " << filepath << std::endl;
        return false;
    }

    const auto& features = doc["features"];
    if (!features.IsArray()) return false;

    glm::dvec3 anchor(ecefAnchor[0], ecefAnchor[1], ecefAnchor[2]);
    uint32_t featureIdCounter = 1;

    for (rapidjson::SizeType i = 0; i < features.Size(); ++i) {
        const auto& feat = features[i];
        if (!feat.IsObject() || !feat.HasMember("geometry")) continue;

        const auto& geom = feat["geometry"];
        if (!geom.IsObject() || !geom.HasMember("type") || !geom.HasMember("coordinates")) continue;

        std::string geomType = geom["type"].GetString();
        if (geomType != "LineString") continue;

        const auto& coords = geom["coordinates"];
        if (!coords.IsArray() || coords.Size() < 2) continue;

        RawRoadFeature road;
        road.id = featureIdCounter++;

        if (feat.HasMember("id") && feat["id"].IsString()) {
            road.osmId = feat["id"].GetString();
        }

        if (feat.HasMember("properties") && feat["properties"].IsObject()) {
            const auto& props = feat["properties"];
            if (props.HasMember("name") && props["name"].IsString()) {
                road.name = props["name"].GetString();
            }
            if (props.HasMember("highway") && props["highway"].IsString()) {
                road.highway = props["highway"].GetString();
            }
            if (props.HasMember("ref") && props["ref"].IsString()) {
                road.ref = props["ref"].GetString();
            }
            if (props.HasMember("destination") && props["destination"].IsString()) {
                road.destination = props["destination"].GetString();
            }
            if (props.HasMember("lanes") && props["lanes"].IsString()) {
                road.lanes = std::max(1, std::atoi(props["lanes"].GetString()));
                road.hasExplicitLanes = true;
            } else {
                road.hasExplicitLanes = false;
                if (road.highway == "trunk") road.lanes = 2;
                else if (road.highway == "trunk_link") road.lanes = 1;
                else road.lanes = 2;
            }
            if (props.HasMember("oneway") && props["oneway"].IsString()) {
                road.oneway = (std::string(props["oneway"].GetString()) == "yes");
            } else {
                road.oneway = (road.highway == "trunk_link");
            }
            if (props.HasMember("width") && props["width"].IsString()) {
                road.parsedWidth = parseDimensionString(props["width"].GetString());
            }
            if (props.HasMember("lane_width") && props["lane_width"].IsString()) {
                road.parsedLaneWidth = parseDimensionString(props["lane_width"].GetString());
            } else if (props.HasMember("width:lanes") && props["width:lanes"].IsString()) {
                road.parsedLaneWidth = parseDimensionString(props["width:lanes"].GetString());
            }
            if (props.HasMember("shoulder") && props["shoulder"].IsString()) {
                road.shoulderTag = props["shoulder"].GetString();
            }
            if (props.HasMember("shoulder:width") && props["shoulder:width"].IsString()) {
                road.parsedShoulderWidth = parseDimensionString(props["shoulder:width"].GetString());
            } else if (props.HasMember("shoulder:width:right") && props["shoulder:width:right"].IsString()) {
                road.parsedShoulderWidth = parseDimensionString(props["shoulder:width"].GetString());
            }
            if (props.HasMember("layer")) {
                if (props["layer"].IsString()) road.layer = std::atoi(props["layer"].GetString());
                else if (props["layer"].IsInt()) road.layer = props["layer"].GetInt();
            }
            if (props.HasMember("bridge") && props["bridge"].IsString()) {
                road.isBridge = (std::string(props["bridge"].GetString()) == "yes");
            }
            if (props.HasMember("tunnel") && props["tunnel"].IsString()) {
                road.isTunnel = (std::string(props["tunnel"].GetString()) == "yes");
            }
        }

        // Base elevation offset: AASHTO standard 5.5m for layer 1 highway overpasses, -5.0m for tunnels, +0.15m for ground
        if (road.isBridge || road.layer > 0) {
            road.elevation = (road.layer > 0 ? (float)road.layer : 1.0f) * 5.5f;
        } else if (road.isTunnel || road.layer < 0) {
            road.elevation = (road.layer < 0 ? (float)road.layer : -1.0f) * 5.0f;
        } else {
            road.elevation = 0.15f; // Sits 15cm on top of terrain
        }

        for (rapidjson::SizeType c = 0; c < coords.Size(); ++c) {
            const auto& pt = coords[c];
            if (pt.IsArray() && pt.Size() >= 2) {
                double lon = pt[0].GetDouble();
                double lat = pt[1].GetDouble();

                glm::dvec3 ecef = TerrainLoader::geodeticToECEF(lat, lon, 0.0);
                glm::vec3 enu = TerrainLoader::ecefToENU(ecef, anchor, enuRotation);

                float x = enu.x;
                float z = -enu.y; // World Z is -North
                float y = road.elevation;

                road.rawPoints.push_back(glm::vec3(x, y, z));
                road.gpsPoints.push_back(glm::dvec2(lon, lat));
            }
        }

        if (road.rawPoints.size() >= 2) {
            outFeatures.push_back(road);
        }
    }

    std::cout << "GeoJsonLoader: Ingested " << outFeatures.size()
              << " LineString road features (WGS84 ECEF Anchor Projected) from " << filepath << std::endl;
    return true;
}

