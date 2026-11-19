#!/usr/bin/env python3
"""
tools/fetch_nj_roads_geofabrik.py

Stream and convert OpenStreetMap New Jersey road network from Geofabrik into GeoJSON:
1. Stream download 'new-jersey-latest-free.shp.zip' (~319 MB) with live progress.
2. Ephemerally extract only 'gis_osm_roads_free_1.*' (.shp and .dbf).
3. Immediately delete raw .zip to strictly bound disk space.
4. Stream binary Shapefile + dBASE attribute table into Standard GeoJSON format.
5. Immediately delete extracted shapefile files upon completion.
"""

import sys
import os
import struct
import json
import zipfile
import urllib.request
import time
from typing import List, Dict, Any, Generator, Tuple

GEOFABRIK_NJ_SHP_URL = "https://download.geofabrik.de/north-america/us/new-jersey-latest-free.shp.zip"

ALLOWED_HIGHWAY_CLASSES = {
    "motorway", "motorway_link",
    "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "unclassified", "residential",
    "living_street", "service"
}

def download_with_progress(url: str, dest_path: str):
    print(f"[DOWNLOAD] Streaming {url} -> {dest_path}")
    os.makedirs(os.path.dirname(os.path.abspath(dest_path)), exist_ok=True)
    
    headers = {"User-Agent": "ForceEngine-GeofabrikDownloader/1.0"}
    req = urllib.request.Request(url, headers=headers)
    t0 = time.time()
    
    with urllib.request.urlopen(req) as resp, open(dest_path, "wb") as f_out:
        total_size = int(resp.headers.get("Content-Length", 0))
        downloaded = 0
        last_log = time.time()
        
        while True:
            chunk = resp.read(65536)
            if not chunk:
                break
            f_out.write(chunk)
            downloaded += len(chunk)
            
            now = time.time()
            if now - last_log >= 2.0 or downloaded == total_size:
                pct = (downloaded / total_size * 100) if total_size > 0 else 0
                mb = downloaded / (1024 * 1024)
                rate = mb / max(0.1, now - t0)
                print(f"  Downloaded: {mb:.1f} MB / {total_size / (1024*1024):.1f} MB ({pct:.1f}%) at {rate:.2f} MB/s", flush=True)
                last_log = now
                
    print(f"[DOWNLOAD] Finished in {time.time() - t0:.1f}s ({os.path.getsize(dest_path) / (1024*1024):.1f} MB).", flush=True)

class DbfReader:
    """Zero-dependency streaming dBASE (.dbf) parser."""
    def __init__(self, dbf_path: str):
        self.file = open(dbf_path, "rb")
        header = self.file.read(32)
        version, y, m, d = struct.unpack_from("<4B", header, 0)
        self.num_records, = struct.unpack_from("<I", header, 4)
        self.header_len, self.record_len = struct.unpack_from("<HH", header, 8)
        
        # Parse field descriptors
        self.fields = []
        bytes_read = 32
        while True:
            desc = self.file.read(32)
            bytes_read += 32
            if not desc or desc[0] == 0x0D:
                break
            name = desc[:11].split(b"\x00")[0].decode("ascii", errors="ignore")
            f_type = chr(desc[11])
            f_len = desc[16]
            self.fields.append((name, f_type, f_len))
            if bytes_read >= self.header_len - 1:
                break
                
        self.file.seek(self.header_len)

    def __iter__(self):
        for _ in range(self.num_records):
            rec_bytes = self.file.read(self.record_len)
            if len(rec_bytes) < self.record_len:
                break
            # First byte is deletion flag: 0x20 is valid, 0x2A is deleted
            if rec_bytes[0] == 0x2A:
                continue
            
            record = {}
            offset = 1
            for name, f_type, f_len in self.fields:
                val_bytes = rec_bytes[offset:offset + f_len]
                offset += f_len
                val_str = val_bytes.decode("utf-8", errors="ignore").strip()
                if f_type == "N":
                    try:
                        record[name] = int(val_str) if val_str else 0
                    except ValueError:
                        record[name] = float(val_str) if val_str else 0.0
                elif f_type == "F":
                    record[name] = float(val_str) if val_str else 0.0
                else:
                    record[name] = val_str
            yield record

    def close(self):
        self.file.close()

class ShpReader:
    """Zero-dependency streaming ESRI Shapefile (.shp) PolyLine parser."""
    def __init__(self, shp_path: str):
        self.file = open(shp_path, "rb")
        header = self.file.read(100)
        file_code, = struct.unpack_from(">I", header, 0)
        if file_code != 9994:
            raise ValueError(f"Invalid Shapefile code: {file_code}")
        version, shape_type = struct.unpack_from("<II", header, 28)
        self.file_length = struct.unpack_from(">I", header, 24)[0] * 2

    def __iter__(self):
        while True:
            rec_hdr = self.file.read(8)
            if len(rec_hdr) < 8:
                break
            rec_num, content_len_words = struct.unpack_from(">II", rec_hdr, 0)
            content_bytes = self.file.read(content_len_words * 2)
            if len(content_bytes) < 4:
                break
            
            stype, = struct.unpack_from("<I", content_bytes, 0)
            if stype == 0:  # Null shape
                yield []
                continue
            if stype != 3:  # PolyLine is 3
                yield []
                continue

            num_parts, num_points = struct.unpack_from("<II", content_bytes, 36)
            parts_offset = 44
            parts = struct.unpack_from(f"<{num_parts}I", content_bytes, parts_offset)
            points_offset = parts_offset + num_parts * 4
            
            # Read all (X, Y) points (double precision)
            raw_pts = struct.unpack_from(f"<{num_points * 2}d", content_bytes, points_offset)
            
            # Split into parts (usually 1 part for road segments)
            part_indices = list(parts) + [num_points]
            lines = []
            for p_idx in range(num_parts):
                s = part_indices[p_idx]
                e = part_indices[p_idx + 1]
                line_coords = []
                for pt in range(s, e):
                    x = raw_pts[pt * 2]
                    y = raw_pts[pt * 2 + 1]
                    line_coords.append((round(x, 7), round(y, 7)))
                lines.append(line_coords)
            yield lines

    def close(self):
        self.file.close()

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Fetch and convert Geofabrik New Jersey roads to GeoJSON")
    parser.add_argument("--out-geojson", default="jughandle/new_jersey_roads.geojson", help="Output GeoJSON path")
    parser.add_argument("--temp-dir", default="jughandle/temp_roads", help="Ephemeral working directory")
    args = parser.parse_args()

    os.makedirs(args.temp_dir, exist_ok=True)
    zip_dest = os.path.join(args.temp_dir, "new-jersey-latest-free.shp.zip")
    
    # 1. Download
    if not os.path.exists(zip_dest):
        download_with_progress(GEOFABRIK_NJ_SHP_URL, zip_dest)
        
    # 2. Extract only road shapefiles
    print("[EXTRACTION] Extracting gis_osm_roads_free_1 files from zip...")
    shp_name = "gis_osm_roads_free_1.shp"
    dbf_name = "gis_osm_roads_free_1.dbf"
    
    with zipfile.ZipFile(zip_dest, "r") as zf:
        zf.extract(shp_name, path=args.temp_dir)
        zf.extract(dbf_name, path=args.temp_dir)
        
    # 3. EPHEMERAL PURGE: Delete zip immediately
    if os.path.exists(zip_dest):
        os.remove(zip_dest)
        print(f"[CLEANUP] Deleted raw zip {zip_dest}. Disk space kept bounded.")

    shp_path = os.path.join(args.temp_dir, shp_name)
    dbf_path = os.path.join(args.temp_dir, dbf_name)

    # 4. Stream convert to GeoJSON
    os.makedirs(os.path.dirname(os.path.abspath(args.out_geojson)), exist_ok=True)
    print(f"[CONVERT] Streaming {shp_name} & {dbf_name} into {args.out_geojson}...")

    t0 = time.time()
    shp_reader = ShpReader(shp_path)
    dbf_reader = DbfReader(dbf_path)

    total_roads = 0
    kept_roads = 0

    with open(args.out_geojson, "w") as f_out:
        f_out.write('{"type": "FeatureCollection", "features": [\n')
        first = True

        for geom_parts, props in zip(shp_reader, dbf_reader):
            total_roads += 1
            fclass = props.get("fclass", "").lower()
            if fclass not in ALLOWED_HIGHWAY_CLASSES:
                continue
            if not geom_parts:
                continue

            for part_coords in geom_parts:
                if len(part_coords) < 2:
                    continue

                feat = {
                    "type": "Feature",
                    "id": str(props.get("osm_id", "")),
                    "properties": {
                        "name": props.get("name", ""),
                        "highway": fclass,
                        "ref": props.get("ref", ""),
                        "layer": str(props.get("layer", 0)),
                        "bridge": "yes" if props.get("bridge") == "B" or props.get("bridge") == "T" else "no",
                        "tunnel": "yes" if props.get("tunnel") == "T" else "no",
                        "oneway": "yes" if props.get("oneway") == "T" or props.get("oneway") == "F" else "no",
                        "maxspeed": str(props.get("maxspeed", 0))
                    },
                    "geometry": {
                        "type": "LineString",
                        "coordinates": part_coords
                    }
                }

                if not first:
                    f_out.write(",\n")
                f_out.write(json.dumps(feat))
                first = False
                kept_roads += 1

                if kept_roads % 50000 == 0:
                    print(f"  Processed {total_roads:,} raw features -> {kept_roads:,} road corridors written...", flush=True)

        f_out.write("\n]}\n")

    shp_reader.close()
    dbf_reader.close()

    print(f"[CONVERT] Completed in {time.time() - t0:.2f}s!")
    print(f"  Total Raw OSM Features: {total_roads:,}")
    print(f"  Kept Driveable Road Features: {kept_roads:,}")
    print(f"  Output GeoJSON Size: {os.path.getsize(args.out_geojson) / (1024*1024):.1f} MB")

    # 5. Clean up extracted shapefile files
    for f in [shp_path, dbf_path]:
        if os.path.exists(f):
            os.remove(f)
            print(f"[CLEANUP] Deleted intermediate {f}")
    try:
        os.rmdir(args.temp_dir)
    except OSError:
        pass

    print("[SUCCESS] New Jersey Statewide Road Network Ready!")

if __name__ == "__main__":
    main()
