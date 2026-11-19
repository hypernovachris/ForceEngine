#!/usr/bin/env python3
"""
ForceEngine - Road Data Acquisition Pipeline
============================================
Fetches OpenStreetMap road network for a specified bounding box or region
using the Overpass API, and converts OSM ways/nodes into structured GeoJSON
compatible with GeoJsonLoader and RoadBaker.
"""

import sys
import os
import json
import urllib.request
import urllib.parse
import time

OVERPASS_SERVERS = [
    "https://overpass-api.de/api/interpreter",
    "https://lz4.overpass-api.de/api/interpreter",
    "https://z.overpass-api.de/api/interpreter",
]

def fetch_osm_highways(bbox, timeout=90):
    """
    bbox: 'minLat,minLon,maxLat,maxLon'
    """
    query = f"""
[out:json][timeout:{timeout}];
(
  way["highway"~"motorway|trunk|primary|secondary|tertiary|residential|unclassified|service|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link"]({bbox});
);
out body;
>;
out skel qt;
"""
    data = urllib.parse.urlencode({"data": query}).encode("utf-8")
    
    last_err = None
    for server in OVERPASS_SERVERS:
        print(f"Querying Overpass server: {server} for bbox {bbox}...")
        try:
            req = urllib.request.Request(
                server,
                data=data,
                headers={"User-Agent": "ForceEngine-Pipeline/1.0 (Automated Road Baker)"}
            )
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw_json = resp.read().decode("utf-8")
                res = json.loads(raw_json)
                elements = res.get("elements", [])
                ways = [e for e in elements if e.get("type") == "way"]
                nodes = [e for e in elements if e.get("type") == "node"]
                print(f"Successfully downloaded {len(ways)} ways, {len(nodes)} nodes from {server}")
                return elements
        except Exception as e:
            print(f"Warning: {server} failed with: {e}")
            last_err = e
            time.sleep(1)
            
    raise RuntimeError(f"All Overpass servers failed. Last error: {last_err}")

def osm_to_geojson(elements):
    """
    Converts OSM elements (nodes + ways) into standard GeoJSON FeatureCollection.
    """
    # 1. Index nodes by ID -> (lon, lat)
    node_coords = {}
    for elem in elements:
        if elem.get("type") == "node":
            node_coords[elem["id"]] = [elem["lon"], elem["lat"]]

    # 2. Convert ways into GeoJSON features
    features = []
    for elem in elements:
        if elem.get("type") != "way":
            continue
        nodes = elem.get("nodes", [])
        coords = [node_coords[nid] for nid in nodes if nid in node_coords]
        if len(coords) < 2:
            continue

        props = dict(elem.get("tags", {}))
        props["osm_id"] = elem["id"]

        feature = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": coords
            },
            "properties": props
        }
        features.append(feature)

    return {
        "type": "FeatureCollection",
        "features": features
    }

def main():
    # Somerset County Terrain Pack bounds:
    # Anchor: lat 40.578, lon -74.6165
    # Extents: +/- 8192m (~16.4 km)
    # Lat: 40.504 to 40.652, Lon: -74.714 to -74.520
    default_bbox = "40.504,-74.714,40.652,-74.520"
    output_path = "jughandle/somerset_roads.geojson"

    if len(sys.argv) > 1:
        bbox = sys.argv[1]
    else:
        bbox = default_bbox

    if len(sys.argv) > 2:
        output_path = sys.argv[2]

    print(f"--- ForceEngine Road Fetcher ---")
    print(f"Target Bounding Box: {bbox}")
    print(f"Target Output Path:  {output_path}")

    elements = fetch_osm_highways(bbox)
    geojson_data = osm_to_geojson(elements)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(geojson_data, f, indent=2)

    print(f"Exported {len(geojson_data['features'])} road features to {output_path} ({os.path.getsize(output_path) / 1024:.1f} KB)")

if __name__ == "__main__":
    main()

