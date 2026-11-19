#!/usr/bin/env python3
"""
ForceEngine - Terrain Fetcher & Hierarchical Quadtree Preprocessor
==================================================================
Fetches high-resolution elevation data (USGS 3DEP 0.5m / 1m bare-earth DEMs & LiDAR),
handles Earth curvature (ECEF Tier A / ENU Tier B & vertex shader horizon drop),
and compiles a hierarchical Quadtree Pyramid into a single .fterrainpack container.

Architecture:
- Base (LOD 0) chunk size: 64m x 64m at 0.5m resolution -> 129 x 129 samples (2^7 + 1).
- Road chunk alignment: 512m x 512m road chunks fit an 8 x 8 grid of LOD 0 terrain chunks,
  and match LOD 3 terrain tiles (64m * 2^3 = 512m) 1:1.
- Quadtree pyramid: Every tile at every LOD level is exactly 129 x 129 samples,
  covering 2x the physical width of its children (LOD 0: 64m, LOD 1: 128m, LOD 2: 256m,
  LOD 3: 512m, LOD 4: 1024m, LOD 5: 2048m, LOD 6: 4096m, LOD 7: 8192m...).
- Single container (.fterrainpack): Eliminates the "Horizon Read Explosion" of thousands of
  loose files. Contains a 192-byte header, a Morton/Z-order sorted index table (44B/entry),
  and a contiguous blob of Deflate-compressed tiles (~33KB uncompressed -> ~8KB on disk).
"""

import sys
import os
import time
import math
import struct
import zlib
import json
import argparse
import urllib.request
import urllib.parse
import concurrent.futures
import threading
from typing import Tuple, List, Dict, Optional, Any

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# Ensure external/lerc/OtherLanguages/Python is in sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
lerc_python_path = os.path.join(repo_root, "external", "lerc", "OtherLanguages", "Python")
if os.path.isdir(lerc_python_path) and lerc_python_path not in sys.path:
    sys.path.insert(0, lerc_python_path)

try:
    import lerc
    HAS_LERC = True
except ImportError:
    HAS_LERC = False

# ==============================================================================
# 1. GEODETIC & EARTH CURVATURE MATH (WGS84, ECEF, ENU)
# ==============================================================================

class WGS84:
    """WGS84 Ellipsoid constants and geodetic coordinate transformations."""
    A = 6378137.0            # Semi-major axis (meters)
    F = 1.0 / 298.257223563  # Flattening
    B = A * (1.0 - F)        # Semi-minor axis (~6356752.314245 m)
    E2 = 2.0 * F - F * F     # First eccentricity squared (~0.00669437999014)
    EP2 = (A * A - B * B) / (B * B) # Second eccentricity squared
    MEAN_RADIUS = 6371000.0  # Spherical Earth radius for curvature drop (meters)

    @classmethod
    def geodetic_to_ecef(cls, lat_deg: float, lon_deg: float, h_meters: float) -> Tuple[float, float, float]:
        """Convert WGS84 (lat, lon, height) to Earth-Centered, Earth-Fixed (ECEF) (X, Y, Z) in meters."""
        phi = math.radians(lat_deg)
        lam = math.radians(lon_deg)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        sin_lam = math.sin(lam)
        cos_lam = math.cos(lam)

        N = cls.A / math.sqrt(1.0 - cls.E2 * sin_phi * sin_phi)
        x = (N + h_meters) * cos_phi * cos_lam
        y = (N + h_meters) * cos_phi * sin_lam
        z = (N * (1.0 - cls.E2) + h_meters) * sin_phi
        return x, y, z

    @classmethod
    def ecef_to_geodetic(cls, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Convert ECEF (X, Y, Z) to WGS84 (lat, lon, height) using Bowring's closed-form algorithm."""
        p = math.hypot(x, y)
        if p < 1e-6:
            lat = 90.0 if z > 0 else -90.0
            return lat, 0.0, abs(z) - cls.B

        theta = math.atan2(z * cls.A, p * cls.B)
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)

        phi = math.atan2(z + cls.EP2 * cls.B * (sin_t ** 3),
                         p - cls.E2 * cls.A * (cos_t ** 3))
        lam = math.atan2(y, x)

        sin_phi = math.sin(phi)
        N = cls.A / math.sqrt(1.0 - cls.E2 * sin_phi * sin_phi)
        h = p / math.cos(phi) - N
        return math.degrees(phi), math.degrees(lam), h

    @classmethod
    def enu_rotation_matrix(cls, lat_deg: float, lon_deg: float) -> List[List[float]]:
        """Compute 3x3 orthonormal rotation matrix R_ENU mapping ECEF deltas to local tangent plane (East, North, Up)."""
        phi = math.radians(lat_deg)
        lam = math.radians(lon_deg)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        sin_lam = math.sin(lam)
        cos_lam = math.cos(lam)

        return [
            [-sin_lam,                cos_lam,               0.0    ],
            [-sin_phi * cos_lam,     -sin_phi * sin_lam,     cos_phi],
            [ cos_phi * cos_lam,      cos_phi * sin_lam,     sin_phi]
        ]

    @classmethod
    def ecef_to_enu(cls, p_ecef: Tuple[float, float, float],
                    origin_ecef: Tuple[float, float, float],
                    R_enu: List[List[float]]) -> Tuple[float, float, float]:
        """Transform a 3D ECEF point into the local ENU tangent frame relative to an origin."""
        dx = p_ecef[0] - origin_ecef[0]
        dy = p_ecef[1] - origin_ecef[1]
        dz = p_ecef[2] - origin_ecef[2]

        e = R_enu[0][0] * dx + R_enu[0][1] * dy + R_enu[0][2] * dz
        n = R_enu[1][0] * dx + R_enu[1][1] * dy + R_enu[1][2] * dz
        u = R_enu[2][0] * dx + R_enu[2][1] * dy + R_enu[2][2] * dz
        return e, n, u

    @classmethod
    def enu_to_ecef(cls, p_enu: Tuple[float, float, float],
                    origin_ecef: Tuple[float, float, float],
                    R_enu: List[List[float]]) -> Tuple[float, float, float]:
        """Inverse transform from local ENU tangent frame back to global ECEF."""
        e, n, u = p_enu
        dx = R_enu[0][0] * e + R_enu[1][0] * n + R_enu[2][0] * u
        dy = R_enu[0][1] * e + R_enu[1][1] * n + R_enu[2][1] * u
        dz = R_enu[0][2] * e + R_enu[1][2] * n + R_enu[2][2] * u
        return origin_ecef[0] + dx, origin_ecef[1] + dy, origin_ecef[2] + dz

    @classmethod
    def earth_curvature_drop(cls, distance_meters: float) -> float:
        """
        Compute optical/physical Earth curvature drop in meters: Drop ≈ d^2 / (2 * R).
        Across 32.768 km (half of 65.5 km tile): Drop ≈ 84.3 meters.
        """
        return (distance_meters * distance_meters) / (2.0 * cls.MEAN_RADIUS)


def morton2d(x: int, y: int) -> int:
    """Interleave bits of two 32-bit coordinates to compute Morton / Z-order curve index."""
    def part1by1(n: int) -> int:
        n &= 0x0000ffff
        n = (n | (n << 8)) & 0x00FF00FF
        n = (n | (n << 4)) & 0x0F0F0F0F
        n = (n | (n << 2)) & 0x33333333
        n = (n | (n << 1)) & 0x55555555
        return n
    return (part1by1(y) << 1) | part1by1(x)


def pack_tile_key(lod: int, tile_x: int, tile_y: int) -> int:
    """Pack (LOD, TileX, TileY) into a 64-bit integer using Morton order."""
    m = morton2d(tile_x, tile_y) & 0x00FFFFFFFFFFFFFF
    return (lod << 56) | m


# ==============================================================================
# 2. PURE-PYTHON GEOTIFF DECODER
# ==============================================================================

class GeoTIFFImage:
    """Decodes TIFF / GeoTIFF floating-point elevation rasters with support for strips & tiles."""
    def __init__(self):
        self.width = 0
        self.height = 0
        self.bits_per_sample = 32
        self.compression = 1
        self.sample_format = 3
        self.min_lon = 0.0
        self.min_lat = 0.0
        self.max_lon = 0.0
        self.max_lat = 0.0
        self.pixel_scale_x = 0.0
        self.pixel_scale_y = 0.0
        self.elevations: List[List[float]] = []
        self.min_elevation = 0.0
        self.max_elevation = 0.0

    @classmethod
    def read_from_file(cls, filepath: str) -> 'GeoTIFFImage':
        with open(filepath, 'rb') as f:
            data = f.read()
        return cls.read_from_bytes(data)

    @classmethod
    def read_from_bytes(cls, data: bytes) -> 'GeoTIFFImage':
        img = cls()
        if len(data) < 8:
            raise ValueError("Invalid TIFF: file too small")

        byte_order = '<' if data[:2] == b'II' else '>'
        version, ifd_offset = struct.unpack_from(byte_order + 'HI', data, 2)
        if version != 42:
            raise ValueError(f"Unsupported TIFF version: {version}")

        num_entries, = struct.unpack_from(byte_order + 'H', data, ifd_offset)

        def read_tag_val(t_type: int, t_count: int, t_val_or_off: int) -> Any:
            type_sizes = { 1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 11: 4, 12: 8 }
            elem_size = type_sizes.get(t_type, 1)
            total_size = elem_size * t_count
            target_offset = t_val_or_off if total_size > 4 else (ifd_offset + 2 + entry_idx * 12 + 8)

            if t_type == 3:
                vals = struct.unpack_from(f"{byte_order}{t_count}H", data, target_offset)
                return vals[0] if t_count == 1 else list(vals)
            elif t_type == 4:
                vals = struct.unpack_from(f"{byte_order}{t_count}I", data, target_offset)
                return vals[0] if t_count == 1 else list(vals)
            elif t_type == 11:
                vals = struct.unpack_from(f"{byte_order}{t_count}f", data, target_offset)
                return vals[0] if t_count == 1 else list(vals)
            elif t_type == 12:
                vals = struct.unpack_from(f"{byte_order}{t_count}d", data, target_offset)
                return vals[0] if t_count == 1 else list(vals)
            elif t_type == 2:
                return data[target_offset:target_offset + t_count].decode('ascii', errors='ignore').strip('\x00')
            return t_val_or_off

        tags: Dict[int, Any] = {}
        for entry_idx in range(num_entries):
            entry_off = ifd_offset + 2 + entry_idx * 12
            tag, tag_type, count, val_or_off = struct.unpack_from(byte_order + 'HHII', data, entry_off)
            tags[tag] = read_tag_val(tag_type, count, val_or_off)

        img.width = tags.get(256, 0)
        img.height = tags.get(257, 0)
        bps = tags.get(258, 32)
        img.bits_per_sample = bps if isinstance(bps, int) else bps[0]
        img.compression = tags.get(259, 1)
        img.sample_format = tags.get(339, 3)

        if 33550 in tags:
            scale = tags[33550]
            img.pixel_scale_x = scale[0]
            img.pixel_scale_y = scale[1]

        if 33922 in tags:
            tp = tags[33922]
            img.min_lon = tp[3]
            img.max_lat = tp[4]
            img.max_lon = img.min_lon + img.width * img.pixel_scale_x
            img.min_lat = img.max_lat - img.height * img.pixel_scale_y

        # Tiled vs Stripped
        if 322 in tags and 324 in tags:
            tile_w = tags[322]
            tile_h = tags[323]
            tile_offsets = tags[324]
            if isinstance(tile_offsets, int): tile_offsets = [tile_offsets]
            tile_byte_counts = tags[325]
            if isinstance(tile_byte_counts, int): tile_byte_counts = [tile_byte_counts]

            tiles_across = (img.width + tile_w - 1) // tile_w
            tiles_down = (img.height + tile_h - 1) // tile_h

            if HAS_NUMPY:
                img.elevations = np.zeros((img.height, img.width), dtype=np.float32)
                for ty in range(tiles_down):
                    for tx in range(tiles_across):
                        tile_idx = ty * tiles_across + tx
                        t_off = tile_offsets[tile_idx]
                        t_bytes = tile_byte_counts[tile_idx]
                        if t_bytes == 0 or t_off == 0:
                            continue # Sparse tile: unallocated background is 0.0 elevation
                        chunk = data[t_off:t_off + t_bytes]
                        if img.compression == 8:
                            chunk = zlib.decompress(chunk)

                        floats = np.frombuffer(chunk, dtype=f"{byte_order}f4").reshape((tile_h, tile_w))
                        floats = np.nan_to_num(floats, nan=0.0)
                        floats[floats <= -99990.0] = 0.0

                        y = ty * tile_h
                        x = tx * tile_w
                        act_h = min(tile_h, img.height - y)
                        act_w = min(tile_w, img.width - x)
                        if act_h > 0 and act_w > 0:
                            img.elevations[y:y + act_h, x:x + act_w] = floats[:act_h, :act_w]
            else:
                img.elevations = [[0.0] * img.width for _ in range(img.height)]
                for ty in range(tiles_down):
                    for tx in range(tiles_across):
                        tile_idx = ty * tiles_across + tx
                        t_off = tile_offsets[tile_idx]
                        t_bytes = tile_byte_counts[tile_idx]
                        if t_bytes == 0 or t_off == 0:
                            continue # Sparse tile: unallocated background is 0.0 elevation
                        chunk = data[t_off:t_off + t_bytes]
                        if img.compression == 8:
                            chunk = zlib.decompress(chunk)

                        num_floats = tile_w * tile_h
                        floats = struct.unpack_from(f"{byte_order}{num_floats}f", chunk, 0)

                        for r in range(tile_h):
                            y = ty * tile_h + r
                            if y >= img.height: break
                            row_start = r * tile_w
                            for c in range(tile_w):
                                x = tx * tile_w + c
                                if x >= img.width: break
                                val = floats[row_start + c]
                                if math.isnan(val) or val <= -99990.0:
                                    val = 0.0
                                img.elevations[y][x] = val

        elif 273 in tags and 279 in tags:
            strip_offsets = tags[273]
            if isinstance(strip_offsets, int): strip_offsets = [strip_offsets]
            strip_byte_counts = tags[279]
            if isinstance(strip_byte_counts, int): strip_byte_counts = [strip_byte_counts]
            rows_per_strip = tags.get(278, img.height)

            if HAS_NUMPY:
                img.elevations = np.zeros((img.height, img.width), dtype=np.float32)
                curr_y = 0
                for s_idx, (s_off, s_bytes) in enumerate(zip(strip_offsets, strip_byte_counts)):
                    num_strip_rows = min(rows_per_strip, img.height - curr_y)
                    if s_bytes == 0 or s_off == 0:
                        curr_y += num_strip_rows
                        continue
                    chunk = data[s_off:s_off + s_bytes]
                    if img.compression == 8:
                        chunk = zlib.decompress(chunk)

                    floats = np.frombuffer(chunk, dtype=f"{byte_order}f4")
                    floats = floats[:num_strip_rows * img.width].reshape((num_strip_rows, img.width))
                    floats = np.nan_to_num(floats, nan=0.0)
                    floats[floats <= -99990.0] = 0.0
                    img.elevations[curr_y:curr_y + num_strip_rows, :] = floats
                    curr_y += num_strip_rows
            else:
                img.elevations = [[0.0] * img.width for _ in range(img.height)]
                curr_y = 0
                for s_idx, (s_off, s_bytes) in enumerate(zip(strip_offsets, strip_byte_counts)):
                    num_strip_rows = min(rows_per_strip, img.height - curr_y)
                    if s_bytes == 0 or s_off == 0:
                        curr_y += num_strip_rows
                        continue
                    chunk = data[s_off:s_off + s_bytes]
                    if img.compression == 8:
                        chunk = zlib.decompress(chunk)

                    floats = struct.unpack_from(f"{byte_order}{len(chunk) // 4}f", chunk, 0)

                    for r in range(num_strip_rows):
                        y = curr_y + r
                        row_start = r * img.width
                        for x in range(img.width):
                            val = floats[row_start + x]
                            if math.isnan(val) or val <= -99990.0:
                                val = 0.0
                            img.elevations[y][x] = val
                    curr_y += num_strip_rows
        else:
            raise ValueError("TIFF contains neither valid tile nor strip offsets")

        if HAS_NUMPY and isinstance(img.elevations, np.ndarray):
            valid = img.elevations > 0.0
            if np.any(valid):
                img.min_elevation = float(np.min(img.elevations[valid]))
                img.max_elevation = float(np.max(img.elevations[valid]))
        else:
            all_vals = [val for row in img.elevations for val in row if val > 0.0]
            if all_vals:
                img.min_elevation = min(all_vals)
                img.max_elevation = max(all_vals)
        return img


# ==============================================================================
# 3. USGS 3DEP ELEVATION CLIENT
# ==============================================================================

class USGS3DEPClient:
    """Client for fetching bare-earth DEMs from USGS 3D Elevation Program services."""
    IMAGE_SERVER_URL = "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
    NJ_LIDAR_EPT_URL = "https://s3-us-west-2.amazonaws.com/usgs-lidar-public/NJ_NW6Co_2017/ept.json"

    @classmethod
    def fetch_dem_image_server(cls, min_lon: float, min_lat: float,
                               max_lon: float, max_lat: float,
                               width: int, height: int,
                               output_filepath: str) -> bool:
        params_base = {
            "bbox": f"{min_lon:.6f},{min_lat:.6f},{max_lon:.6f},{max_lat:.6f}",
            "bboxSR": "4326",
            "imageSR": "4326",
            "size": f"{width},{height}",
            "format": "tiff",
            "pixelType": "F32",
        }
        print(f"[USGS 3DEP] Requesting DEM from ImageServer:")
        print(f"  BBox: [{min_lon:.5f}, {min_lat:.5f}] -> [{max_lon:.5f}, {max_lat:.5f}]")
        print(f"  Dimensions: {width} x {height} px")
        print(f"  Target: {output_filepath}")

        os.makedirs(os.path.dirname(os.path.abspath(output_filepath)), exist_ok=True)
        headers = {"User-Agent": "ForceEngine-TerrainProcessor/1.0"}

        href_download_url = None
        # Try f=json first (generates on server and returns direct href URL)
        params_json = dict(params_base)
        params_json["f"] = "json"
        url_json = cls.IMAGE_SERVER_URL + "?" + urllib.parse.urlencode(params_json)

        try:
            print(f"[USGS 3DEP] Requesting server-side raster generation (f=json)...")
            req = urllib.request.Request(url_json, headers=headers)
            with urllib.request.urlopen(req, timeout=120) as resp:
                resp_text = resp.read().decode('utf-8', errors='ignore')
                try:
                    data_obj = json.loads(resp_text)
                    if "href" in data_obj:
                        href_download_url = data_obj["href"]
                        print(f"[USGS 3DEP] Server generated raster: {href_download_url}")
                    elif "error" in data_obj:
                        print(f"[USGS 3DEP] Server notice: {data_obj['error']}")
                except json.JSONDecodeError:
                    pass
        except Exception as e:
            print(f"[USGS 3DEP] Note: json export call failed ({e}), falling back to direct stream...")

        if href_download_url:
            download_url = href_download_url
        else:
            params_img = dict(params_base)
            params_img["f"] = "image"
            download_url = cls.IMAGE_SERVER_URL + "?" + urllib.parse.urlencode(params_img)

        # Stream download with progress
        try:
            print(f"[USGS 3DEP] Streaming GeoTIFF data...")
            req = urllib.request.Request(download_url, headers=headers)
            with urllib.request.urlopen(req, timeout=180) as resp:
                total_header = resp.headers.get('Content-Length')
                total_size = int(total_header) if total_header else 0
                downloaded = 0
                chunk_size = 1024 * 1024 # 1 MB chunks

                with open(output_filepath, "wb") as f:
                    while True:
                        chunk = resp.read(chunk_size)
                        if not chunk:
                            break
                        f.write(chunk)
                        downloaded += len(chunk)
                        if total_size > 0:
                            pct = (downloaded / total_size) * 100.0
                            print(f"\r  Downloaded: {downloaded / (1024*1024):.1f} MB / {total_size / (1024*1024):.1f} MB ({pct:.1f}%)", end="", flush=True)
                        else:
                            print(f"\r  Downloaded: {downloaded / (1024*1024):.1f} MB", end="", flush=True)
                print()

                if downloaded < 200:
                    print(f"[USGS 3DEP] Error response too small ({downloaded} bytes). Check URL / parameters.")
                    return False
                print(f"[USGS 3DEP] Successfully fetched {downloaded:,} bytes -> {output_filepath}")
                return True
        except Exception as e:
            print(f"\n[USGS 3DEP] Fetch failed: {e}")
            return False

    @classmethod
    def fetch_dem_lerc(cls, min_lon: float, min_lat: float,
                       max_lon: float, max_lat: float,
                       width: int, height: int) -> Tuple[bool, Optional['GeoTIFFImage']]:
        """Fetch DEM compressed as LERC directly from ImageServer into memory (zero intermediate disk footprint)."""
        params = {
            "bbox": f"{min_lon:.6f},{min_lat:.6f},{max_lon:.6f},{max_lat:.6f}",
            "bboxSR": "4326",
            "imageSR": "4326",
            "size": f"{width},{height}",
            "format": "lerc",
            "f": "image"
        }
        url = cls.IMAGE_SERVER_URL + "?" + urllib.parse.urlencode(params)
        headers = {"User-Agent": "ForceEngine-TerrainProcessor/1.0"}
        for attempt in range(5):
            try:
                req = urllib.request.Request(url, headers=headers)
                with urllib.request.urlopen(req, timeout=90) as resp:
                    data = resp.read()
                if len(data) < 200:
                    return False, None
                if not HAS_LERC:
                    return False, None
                res, arr, mask = lerc.decode(data)
                if res != 0 or arr is None:
                    return False, None
                if mask is not None and not np.any(mask):
                    return False, None
                if np.all(arr <= 0.01):
                    return False, None

                # Account for ArcGIS ImageServer aspect ratio adjustment
                req_aspect = (max_lon - min_lon) / (max_lat - min_lat)
                img_aspect = float(width) / float(height)
                act_min_lon, act_max_lon = min_lon, max_lon
                act_min_lat, act_max_lat = min_lat, max_lat
                if abs(req_aspect - img_aspect) > 1e-4:
                    c_lon = 0.5 * (min_lon + max_lon)
                    c_lat = 0.5 * (min_lat + max_lat)
                    if req_aspect > img_aspect:
                        new_span_lat = (max_lon - min_lon) / img_aspect
                        act_min_lat = c_lat - 0.5 * new_span_lat
                        act_max_lat = c_lat + 0.5 * new_span_lat
                    else:
                        new_span_lon = (max_lat - min_lat) * img_aspect
                        act_min_lon = c_lon - 0.5 * new_span_lon
                        act_max_lon = c_lon + 0.5 * new_span_lon

                img = GeoTIFFImage()
                img.width = width
                img.height = height
                img.min_lon = act_min_lon
                img.max_lon = act_max_lon
                img.min_lat = act_min_lat
                img.max_lat = act_max_lat
                img.origin_lon = act_min_lon
                img.origin_lat = act_max_lat
                img.pixel_scale_x = (act_max_lon - act_min_lon) / width
                img.pixel_scale_y = (act_max_lat - act_min_lat) / height
                img.elevations = arr
                valid = arr > 0.0
                img.min_elevation = float(np.min(arr[valid])) if np.any(valid) else 0.0
                img.max_elevation = float(np.max(arr[valid])) if np.any(valid) else 0.0
                return True, img
            except Exception as e:
                if attempt == 4:
                    print(f"  [USGS Error] Failed after 5 attempts ({e}). Skipping.")
                    return False, None
                time.sleep(2.0 * (attempt + 1))
        return False, None

    @classmethod
    def generate_pdal_pipeline_json(cls, ept_url: str,
                                     min_x: float, min_y: float,
                                     max_x: float, max_y: float,
                                     resolution: float,
                                     output_tif: str) -> str:
        pipeline = {
            "pipeline": [
                {
                    "type": "readers.ept",
                    "filename": ept_url,
                    "bounds": f"([{min_x},{max_x}],[{min_y},{max_y}])"
                },
                {
                    "type": "filters.range",
                    "limits": "Classification[2:2]"
                },
                {
                    "type": "writers.gdal",
                    "filename": output_tif,
                    "resolution": resolution,
                    "output_type": "idw",
                    "data_type": "float32"
                }
            ]
        }
        return json.dumps(pipeline, indent=2)


# ==============================================================================
# 4. HIERARCHICAL QUADTREE PYRAMID PREPROCESSOR (.fterrainpack)
# ==============================================================================

def bilinear_upsample_quadrant(parent_elevs_2d: np.ndarray, quad_x: int, quad_y: int) -> np.ndarray:
    """
    Given a 129x129 parent tile and a quadrant (quad_x in {0, 1}, quad_y in {0, 1}):
    Bilinearly interpolate the parent's quadrant into a 129x129 child elevation grid.
    Quadrant (0, 0): [0..64, 0..64] (South-West)
    Quadrant (1, 0): [64..128, 0..64] (South-East)
    Quadrant (0, 1): [0..64, 64..128] (North-West)
    Quadrant (1, 1): [64..128, 64..128] (North-East)
    """
    ci = np.arange(129, dtype=np.float32)
    cj = np.arange(129, dtype=np.float32)
    px = (quad_x * 64.0) + ci * 0.5
    py = (quad_y * 64.0) + cj * 0.5

    ix = np.floor(px).astype(np.int32)
    iy = np.floor(py).astype(np.int32)
    tx = px - ix
    ty = py - iy

    ix1 = np.minimum(ix + 1, 128)
    iy1 = np.minimum(iy + 1, 128)

    p00 = parent_elevs_2d[iy[:, None], ix[None, :]]
    p10 = parent_elevs_2d[iy[:, None], ix1[None, :]]
    p01 = parent_elevs_2d[iy1[:, None], ix[None, :]]
    p11 = parent_elevs_2d[iy1[:, None], ix1[None, :]]

    return (p00 * (1.0 - tx[None, :]) + p10 * tx[None, :]) * (1.0 - ty[:, None]) + \
           (p01 * (1.0 - tx[None, :]) + p11 * tx[None, :]) * ty[:, None]


class HierarchicalTerrainPacker:
    """
    Builds a hierarchical quadtree pyramid of 129x129 tiles into a single .fterrainpack file.
    Base LOD 0 is base_tile_size x base_tile_size.
    LOD k covers (base_tile_size * 2^k) x (base_tile_size * 2^k).
    Tiles are stored in Morton Z-order curve and compressed via Deflate.
    """
    PACK_HEADER_SIZE = 192
    INDEX_ENTRY_SIZE = 44
    SAMPLES_PER_TILE = 129 # 128 quads + 1 boundary vertex (2^7 + 1)
    BASE_TILE_SIZE = 64.0  # default fallback meters
    BASE_RESOLUTION = 0.5  # default fallback meters per sample

    def __init__(self, geo_img: GeoTIFFImage, base_tile_size: float = 64.0):
        self.img = geo_img
        self.base_tile_size = float(base_tile_size)
        self.base_resolution = self.base_tile_size / (self.SAMPLES_PER_TILE - 1)
        self.center_lon = (geo_img.min_lon + geo_img.max_lon) * 0.5
        self.center_lat = (geo_img.min_lat + geo_img.max_lat) * 0.5
        self.center_ecef = WGS84.geodetic_to_ecef(self.center_lat, self.center_lon, 0.0)
        self.R_enu = WGS84.enu_rotation_matrix(self.center_lat, self.center_lon)

        sw_ecef = WGS84.geodetic_to_ecef(geo_img.min_lat, geo_img.min_lon, 0.0)
        se_ecef = WGS84.geodetic_to_ecef(geo_img.min_lat, geo_img.max_lon, 0.0)
        nw_ecef = WGS84.geodetic_to_ecef(geo_img.max_lat, geo_img.min_lon, 0.0)

        e_width, _, _ = WGS84.ecef_to_enu(se_ecef, sw_ecef, self.R_enu)
        _, n_height, _ = WGS84.ecef_to_enu(nw_ecef, sw_ecef, self.R_enu)

        self.metric_width = abs(e_width)
        self.metric_height = abs(n_height)

    def sample_elevation_bilinear(self, u: float, v: float) -> float:
        u = max(0.0, min(1.0, u))
        v = max(0.0, min(1.0, v))
        fx = u * (self.img.width - 1)
        fy = v * (self.img.height - 1)
        x0 = int(math.floor(fx))
        y0 = int(math.floor(fy))
        x1 = min(x0 + 1, self.img.width - 1)
        y1 = min(y0 + 1, self.img.height - 1)
        tx = fx - x0
        ty = fy - y0

        row0 = self.img.elevations[y0]
        row1 = self.img.elevations[y1]
        h0 = row0[x0] * (1.0 - tx) + row0[x1] * tx
        h1 = row1[x0] * (1.0 - tx) + row1[x1] * tx
        return h0 * (1.0 - ty) + h1 * ty

    def sample_tile(self, min_east: float, min_north: float, tile_size: float):
        """
        Sample a 129x129 elevation grid covering [min_east, min_east + tile_size]
        and [min_north, min_north + tile_size].
        Sample 0 is at min_east, sample 128 is at min_east + tile_size (exact overlap with neighbor!).
        """
        dim = self.SAMPLES_PER_TILE
        step = tile_size / (dim - 1)
        half_w = self.metric_width * 0.5
        half_h = self.metric_height * 0.5

        if HAS_NUMPY and isinstance(self.img.elevations, np.ndarray):
            j_arr = np.arange(dim, dtype=np.float32)
            i_arr = np.arange(dim, dtype=np.float32)
            north_arr = min_north + j_arr * step
            east_arr = min_east + i_arr * step

            u_arr = np.clip((east_arr + half_w) / self.metric_width, 0.0, 1.0)
            v_arr = np.clip(1.0 - ((north_arr + half_h) / self.metric_height), 0.0, 1.0)

            fx = u_arr * (self.img.width - 1)
            fy = v_arr * (self.img.height - 1)

            x0 = np.floor(fx).astype(np.int32)
            y0 = np.floor(fy).astype(np.int32)
            x1 = np.minimum(x0 + 1, self.img.width - 1)
            y1 = np.minimum(y0 + 1, self.img.height - 1)

            tx = fx - x0
            ty = fy - y0

            h00 = self.img.elevations[y0[:, None], x0[None, :]]
            h01 = self.img.elevations[y0[:, None], x1[None, :]]
            h10 = self.img.elevations[y1[:, None], x0[None, :]]
            h11 = self.img.elevations[y1[:, None], x1[None, :]]

            h0 = h00 * (1.0 - tx[None, :]) + h01 * tx[None, :]
            h1 = h10 * (1.0 - tx[None, :]) + h11 * tx[None, :]
            elevs_2d = h0 * (1.0 - ty[:, None]) + h1 * ty[:, None]
            return elevs_2d.flatten()

        elevs = [0.0] * (dim * dim)
        for j in range(dim):
            north = min_north + j * step
            v = 1.0 - ((north + half_h) / self.metric_height)
            row_idx = j * dim
            for i in range(dim):
                east = min_east + i * step
                u = (east + half_w) / self.metric_width
                elevs[row_idx + i] = self.sample_elevation_bilinear(u, v)

        return elevs

    def build_pack_container(self, output_pack_filepath: str,
                            max_lod: Optional[int] = None,
                            format_type: str = "float32",
                            compression: str = "lerc",
                            prune_tolerance: float = 0.02,
                            lerc_tolerance: float = 0.015) -> Dict[str, Any]:
        """
        Constructs the quadtree pyramid from LOD 0 up to max_lod,
        performs adaptive quadtree pruning (pruning children if max residual error <= prune_tolerance),
        compresses retained tiles using LERC (or Deflate fallback),
        sorts tiles by Morton Z-order, and writes a single .fterrainpack container.
        """
        if compression == "lerc" and not HAS_LERC:
            print("[WARNING] LERC requested but lerc module not available. Falling back to Deflate.")
            compression = "deflate"

        if max_lod is None:
            max_lod = 5

        root_sz = self.base_tile_size * (2 ** max_lod)
        rx_count = max(1, int(math.ceil(self.metric_width / root_sz)))
        ry_count = max(1, int(math.ceil(self.metric_height / root_sz)))

        tiles_x_lod0 = rx_count * (2 ** max_lod)
        tiles_y_lod0 = ry_count * (2 ** max_lod)

        print(f"=== Compiling Hierarchical Quadtree Pyramid ===")
        print(f"  Base LOD 0: {self.base_tile_size:.1f}m x {self.base_tile_size:.1f}m ({self.SAMPLES_PER_TILE}x{self.SAMPLES_PER_TILE} samples, {self.base_resolution:.2f}m resolution)")
        print(f"  LOD 0 Grid: {tiles_x_lod0} x {tiles_y_lod0} = {tiles_x_lod0 * tiles_y_lod0} max leaf tiles")
        print(f"  Roots at LOD {max_lod} ({root_sz:.0f}m): {rx_count} x {ry_count} = {rx_count * ry_count} root chunks")
        print(f"  LOD Range: 0 to {max_lod}")
        print(f"  Compression: {compression.upper()} (tolerance: {lerc_tolerance*100:.1f}cm)")
        print(f"  Adaptive Pruning Tolerance: {prune_tolerance*100:.1f}cm ({'Enabled' if prune_tolerance > 0 else 'Disabled'})")

        generated_tiles = []
        start_e = - (tiles_x_lod0 * self.base_tile_size) * 0.5
        start_n = - (tiles_y_lod0 * self.base_tile_size) * 0.5

        total_uncompressed_bytes = 0
        total_compressed_bytes = 0
        lod_tile_counts = {l: 0 for l in range(max_lod + 1)}
        total_pruned_tiles = 0
        total_evaluated_tiles = 0

        # Process root chunk by root chunk to bound working memory
        for ry in range(ry_count):
            for rx in range(rx_count):
                chunk_tiles = {} # lod -> {(tx, ty): 2d numpy array}

                # 1. Sample all levels in this root chunk
                for lod in range(max_lod + 1):
                    tile_sz = self.base_tile_size * (2 ** lod)
                    factor = 2 ** (max_lod - lod)
                    tx_start = rx * factor
                    ty_start = ry * factor
                    chunk_tiles[lod] = {}

                    for dy in range(factor):
                        ty = ty_start + dy
                        tile_min_n = start_n + ty * tile_sz
                        for dx in range(factor):
                            tx = tx_start + dx
                            tile_min_e = start_e + tx * tile_sz
                            elevs = self.sample_tile(tile_min_e, tile_min_n, tile_sz)
                            if HAS_NUMPY and isinstance(elevs, np.ndarray):
                                chunk_tiles[lod][(tx, ty)] = elevs.reshape((self.SAMPLES_PER_TILE, self.SAMPLES_PER_TILE))
                            else:
                                chunk_tiles[lod][(tx, ty)] = np.array(elevs, dtype=np.float32).reshape((self.SAMPLES_PER_TILE, self.SAMPLES_PER_TILE))
                            total_evaluated_tiles += 1

                # 2. Bottom-up Adaptive Quadtree Pruning
                pruned_in_chunk = set() # set of (lod, tx, ty)
                if prune_tolerance > 0 and HAS_NUMPY:
                    # Leaf candidate keys at LOD 0
                    factor0 = 2 ** max_lod
                    leaf_candidates = {(rx * factor0 + dx, ry * factor0 + dy) for dy in range(factor0) for dx in range(factor0)}

                    for lod in range(1, max_lod + 1):
                        factor = 2 ** (max_lod - lod)
                        tx_start = rx * factor
                        ty_start = ry * factor

                        for dy in range(factor):
                            ty = ty_start + dy
                            for dx in range(factor):
                                tx = tx_start + dx
                                p_key = (tx, ty)
                                parent_elevs = chunk_tiles[lod][p_key]

                                c00 = (2 * tx,     2 * ty)
                                c10 = (2 * tx + 1, 2 * ty)
                                c01 = (2 * tx,     2 * ty + 1)
                                c11 = (2 * tx + 1, 2 * ty + 1)
                                c_keys = [c00, c10, c01, c11]

                                # All 4 children must be unpruned leaves at lod - 1
                                if not all(k in leaf_candidates and (lod - 1, k[0], k[1]) not in pruned_in_chunk for k in c_keys):
                                    continue

                                # Check residual error for all 4 quadrants
                                quads = [(0, 0), (1, 0), (0, 1), (1, 1)]
                                can_prune = True
                                for (qx, qy), c_key in zip(quads, c_keys):
                                    child_elevs = chunk_tiles[lod - 1][c_key]
                                    interp = bilinear_upsample_quadrant(parent_elevs, qx, qy)
                                    err = float(np.max(np.abs(child_elevs - interp)))
                                    if err > prune_tolerance:
                                        can_prune = False
                                        break

                                if can_prune:
                                    for c_key in c_keys:
                                        pruned_in_chunk.add((lod - 1, c_key[0], c_key[1]))
                                        leaf_candidates.remove(c_key)
                                    leaf_candidates.add(p_key)

                total_pruned_tiles += len(pruned_in_chunk)

                # 3. Compress and record all kept tiles
                for lod in range(max_lod + 1):
                    factor = 2 ** (max_lod - lod)
                    tx_start = rx * factor
                    ty_start = ry * factor

                    for dy in range(factor):
                        ty = ty_start + dy
                        for dx in range(factor):
                            tx = tx_start + dx
                            if (lod, tx, ty) in pruned_in_chunk:
                                continue # Skipped due to adaptive pruning!

                            elevs_2d = chunk_tiles[lod][(tx, ty)]
                            min_h = float(np.min(elevs_2d))
                            max_h = float(np.max(elevs_2d))

                            if compression == "lerc" and HAS_LERC:
                                if format_type == "float32":
                                    # LERC float32 (Format = 2)
                                    elevs_f32 = np.ascontiguousarray(elevs_2d, dtype=np.float32)
                                    res, nbytes, compressed_payload = lerc.encode(elevs_f32, 1, False, None, lerc_tolerance, 1)
                                    uncompressed_len = elevs_f32.nbytes
                                    fmt_id = 2
                                else:
                                    # LERC uint16 (Format = 3)
                                    rng = max_h - min_h
                                    if rng < 1e-4:
                                        raw16 = np.zeros(elevs_2d.shape, dtype=np.uint16)
                                        u16_tol = 1.0
                                    else:
                                        inv_rng = 65535.0 / rng
                                        raw16 = np.clip((elevs_2d - min_h) * inv_rng + 0.5, 0, 65535).astype(np.uint16)
                                        u16_tol = max(1.0, lerc_tolerance * inv_rng)
                                    raw16 = np.ascontiguousarray(raw16)
                                    res, nbytes, compressed_payload = lerc.encode(raw16, 1, False, None, u16_tol, 1)
                                    uncompressed_len = raw16.nbytes
                                    fmt_id = 3
                            else:
                                # Deflate (Format = 0 uint16, Format = 1 float32)
                                if format_type == "uint16":
                                    rng = max_h - min_h
                                    if rng < 1e-4:
                                        raw16 = np.zeros(elevs_2d.shape, dtype=np.uint16)
                                    else:
                                        inv_rng = 65535.0 / rng
                                        raw16 = np.clip((elevs_2d - min_h) * inv_rng + 0.5, 0, 65535).astype(np.uint16)
                                    uncompressed_payload = raw16.tobytes()
                                    fmt_id = 0
                                else:
                                    uncompressed_payload = elevs_2d.astype(np.float32).tobytes()
                                    fmt_id = 1

                                uncompressed_len = len(uncompressed_payload)
                                compressed_payload = zlib.compress(uncompressed_payload, level=6)

                            total_uncompressed_bytes += uncompressed_len
                            total_compressed_bytes += len(compressed_payload)
                            lod_tile_counts[lod] += 1

                            m_key = pack_tile_key(lod, tx, ty)
                            generated_tiles.append({
                                "key": m_key,
                                "lod": lod,
                                "tx": tx,
                                "ty": ty,
                                "min_h": min_h,
                                "max_h": max_h,
                                "fmt": fmt_id,
                                "uncompressed_sz": uncompressed_len,
                                "compressed_sz": len(compressed_payload),
                                "data": compressed_payload
                            })

        print(f"  Adaptive Pruning Results: {total_pruned_tiles:,} tiles pruned ({total_pruned_tiles / max(1, total_evaluated_tiles) * 100:.1f}%), {len(generated_tiles):,} tiles kept")

        # Sort all tiles in Morton / Z-order curve:
        generated_tiles.sort(key=lambda t: t["key"])

        # Layout Pack File
        num_tiles = len(generated_tiles)
        index_offset = self.PACK_HEADER_SIZE
        index_bytes = num_tiles * self.INDEX_ENTRY_SIZE
        data_offset = index_offset + index_bytes

        print(f"\nWriting single container pack file: {output_pack_filepath}")
        print(f"  Total tiles: {num_tiles:,}")
        print(f"  Uncompressed data: {total_uncompressed_bytes / (1024*1024):.2f} MB")
        print(f"  Compressed data on disk: {total_compressed_bytes / (1024*1024):.2f} MB ({total_compressed_bytes / max(1, total_uncompressed_bytes) * 100:.1f}%)")

        os.makedirs(os.path.dirname(os.path.abspath(output_pack_filepath)), exist_ok=True)

        with open(output_pack_filepath, "wb") as f:
            # 1. Pack Header (192 bytes)
            magic = b"FTPK"
            version = 2
            # Flags: Bit 0: IsUint16, Bit 1: Deflate, Bit 2: LERC
            flags = (1 if format_type == "uint16" else 0) | (4 if compression == "lerc" else 2)
            enu_flat = [float(val) for row in self.R_enu for val in row]

            hdr_full = bytearray(self.PACK_HEADER_SIZE)
            struct.pack_into("<4sIfIf5IQQff3d2d9ffI44x", hdr_full, 0,
                             magic, version,
                             self.base_tile_size, self.SAMPLES_PER_TILE,
                             self.base_resolution,
                             0, max_lod, num_tiles,
                             0, 0, # reserved0, reserved1
                             index_offset, data_offset,
                             self.img.min_elevation, self.img.max_elevation,
                             self.center_ecef[0], self.center_ecef[1], self.center_ecef[2],
                             self.center_lat, self.center_lon,
                             *enu_flat,
                             WGS84.MEAN_RADIUS, flags)
            assert len(hdr_full) == 192, f"Header size mismatch: {len(hdr_full)} != 192"
            f.write(hdr_full)

            # 2. Write Morton Index Table (num_tiles * 44 bytes)
            curr_data_offset = data_offset
            for t in generated_tiles:
                entry = struct.pack("<QIIHHQIIff",
                                    t["key"], t["tx"], t["ty"],
                                    t["lod"], t["fmt"],
                                    curr_data_offset,
                                    t["compressed_sz"],
                                    t["uncompressed_sz"],
                                    t["min_h"], t["max_h"])
                f.write(entry)
                curr_data_offset += t["compressed_sz"]

            # 3. Write Contiguous Blob of Compressed Tiles
            for t in generated_tiles:
                f.write(t["data"])

        # Write companion summary JSON
        summary = {
            "format": "ForceEngine-TerrainPack",
            "version": 2,
            "container_file": os.path.basename(output_pack_filepath),
            "base_tile_size_meters": self.base_tile_size,
            "samples_per_tile": self.SAMPLES_PER_TILE,
            "base_resolution_meters": self.base_resolution,
            "lod_range": [0, max_lod],
            "road_chunk_alignment": f"LOD 3 = {self.base_tile_size * 8:.0f}m",
            "compression": compression.upper(),
            "format_type": format_type,
            "prune_tolerance_meters": prune_tolerance,
            "lerc_tolerance_meters": lerc_tolerance,
            "pruned_tiles": total_pruned_tiles,
            "total_tiles": num_tiles,
            "tiles_per_lod": lod_tile_counts,
            "file_size_bytes": os.path.getsize(output_pack_filepath),
            "uncompressed_bytes": total_uncompressed_bytes,
            "compression_ratio": round(total_uncompressed_bytes / max(1, total_compressed_bytes), 2),
            "geodetic_center": { "lat": self.center_lat, "lon": self.center_lon },
            "ecef_origin": list(self.center_ecef),
            "elevation_range": [self.img.min_elevation, self.img.max_elevation]
        }

        meta_file = os.path.splitext(output_pack_filepath)[0] + "_manifest.json"
        with open(meta_file, "w") as f:
            json.dump(summary, f, indent=2)

        return summary


# ==============================================================================
# 5. CLI & WORKFLOW ORCHESTRATION
# ==============================================================================

def cmd_fetch(args):
    output_dir = args.out_dir
    os.makedirs(output_dir, exist_ok=True)
    out_file = os.path.join(output_dir, args.out_filename)

    min_lon, min_lat, max_lon, max_lat = args.bbox
    print(f"=== 1. Fetching USGS 3DEP Elevation Data ===")
    print(f"BBox: [{min_lon:.5f}, {min_lat:.5f}] -> [{max_lon:.5f}, {max_lat:.5f}]")
    print(f"Requested resolution: {args.resolution} m/px")

    mid_lat = (min_lat + max_lat) * 0.5
    meters_per_deg_lat = 111045.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(mid_lat))

    span_x_m = (max_lon - min_lon) * meters_per_deg_lon
    span_y_m = (max_lat - min_lat) * meters_per_deg_lat

    width_px = max(64, min(8000, int(math.ceil(span_x_m / args.resolution))))
    height_px = max(64, min(8000, int(math.ceil(span_y_m / args.resolution))))

    print(f"Span: {span_x_m:,.1f}m E-W x {span_y_m:,.1f}m N-S -> {width_px} x {height_px} raster pixels")

    ok = USGS3DEPClient.fetch_dem_image_server(min_lon, min_lat, max_lon, max_lat,
                                               width_px, height_px, out_file)
    if not ok:
        print("Failed to download DEM from USGS 3DEP.")
        sys.exit(1)
    return out_file


def cmd_preprocess(args, dem_filepath: Optional[str] = None):
    dem_file = dem_filepath or args.input
    if not os.path.exists(dem_file):
        print(f"Input file not found: {dem_file}")
        sys.exit(1)

    base_tile_sz = getattr(args, 'base_tile_size', None)
    if base_tile_sz is None:
        if args.region_name == "new_jersey" or "new_jersey" in dem_file:
            base_tile_sz = 4096.0
        else:
            base_tile_sz = 64.0

    max_lod = args.max_lod
    if max_lod is None:
        if base_tile_sz >= 4096.0:
            max_lod = 5
        elif base_tile_sz >= 2048.0:
            max_lod = 6
        else:
            max_lod = 5

    compression = getattr(args, 'compression', 'lerc')
    prune_tol = getattr(args, 'prune_tolerance', 0.02)
    lerc_tol = getattr(args, 'lerc_tolerance', 0.015)
    fmt_type = getattr(args, 'format', 'float32')

    print(f"\n=== 2. Preprocessing Elevation Raster into Quadtree Pack ===")
    print(f"Reading GeoTIFF: {dem_file}")
    geo_img = GeoTIFFImage.read_from_file(dem_file)

    packer = HierarchicalTerrainPacker(geo_img, base_tile_size=base_tile_sz)
    pack_file = args.out_pack or os.path.join(args.out_dir, f"{args.region_name}.fterrainpack")

    summary = packer.build_pack_container(pack_file, max_lod=max_lod,
                                          format_type=fmt_type,
                                          compression=compression,
                                          prune_tolerance=prune_tol,
                                          lerc_tolerance=lerc_tol)
    print(f"\n[SUCCESS] Generated single binary container -> {pack_file}")
    print(f"Manifest written to -> {os.path.splitext(pack_file)[0] + '_manifest.json'}")


INDEX_DTYPE = np.dtype([
    ('tileKey', '<u8'),
    ('tileX', '<u4'),
    ('tileY', '<u4'),
    ('lod', '<u2'),
    ('format', '<u2'),
    ('byteOffset', '<u8'),
    ('compressedSize', '<u4'),
    ('uncompressedSize', '<u4'),
    ('minElevation', '<f4'),
    ('maxElevation', '<f4')
])


def process_single_megachunk(mx: int, my: int, mx_idx: int, my_idx: int,
                             mc_sz: float, pad: float,
                             center_ecef: Tuple[float, float, float],
                             R_enu: List[List[float]],
                             args, offline_source_img: Optional[GeoTIFFImage],
                             temp_dir: str):
    """Download and compile a single 8192m root megachunk (LOD 0..7). Returns scalar summary and writes mc_{mx}_{my}.bin."""
    chunk_bin = os.path.join(temp_dir, f"mc_{mx}_{my}.bin")
    ocean_marker = os.path.join(temp_dir, f"mc_{mx}_{my}.ocean")

    # Fast resume from existing disk cache
    if os.path.exists(ocean_marker):
        return mx, my, mx_idx, my_idx, 0, 0.0, 0.0, 0, 0, 0, 0, {}

    if os.path.exists(chunk_bin):
        try:
            with open(chunk_bin, "rb") as f_c:
                hdr_bytes = f_c.read(32)
                if len(hdr_bytes) == 32:
                    magic, num_t, min_h, max_h, prn_c, ev_c, unc_b, cmp_b = struct.unpack("<4sIffIIII", hdr_bytes)
                    if magic == b"MCHK":
                        idx_bytes = f_c.read(num_t * 44)
                        lod_arr = np.frombuffer(idx_bytes, dtype=INDEX_DTYPE)['lod']
                        unique, counts = np.unique(lod_arr, return_counts=True)
                        lod_counts = dict(zip(unique.tolist(), counts.tolist()))
                        return mx, my, mx_idx, my_idx, num_t, min_h, max_h, prn_c, ev_c, unc_b, cmp_b, lod_counts
        except Exception:
            pass

    try:
        mc_min_e = mx * mc_sz
        mc_min_n = my * mc_sz

        p_sw_m = WGS84.enu_to_ecef((mc_min_e - pad, mc_min_n - pad, 0.0), center_ecef, R_enu)
        p_ne_m = WGS84.enu_to_ecef((mc_min_e + mc_sz + pad, mc_min_n + mc_sz + pad, 0.0), center_ecef, R_enu)
        geo_sw_lat, geo_sw_lon, _ = WGS84.ecef_to_geodetic(*p_sw_m)
        geo_ne_lat, geo_ne_lon, _ = WGS84.ecef_to_geodetic(*p_ne_m)

        c_min_lon = min(geo_sw_lon, geo_ne_lon)
        c_max_lon = max(geo_sw_lon, geo_ne_lon)
        c_min_lat = min(geo_sw_lat, geo_ne_lat)
        c_max_lat = max(geo_sw_lat, geo_ne_lat)

        if offline_source_img is not None:
            geo_img = offline_source_img
        else:
            # Native LERC fetch: 4096px over 8192m is 2.0m sample spacing from USGS bare-earth
            ok, geo_img = USGS3DEPClient.fetch_dem_lerc(c_min_lon, c_min_lat, c_max_lon, c_max_lat, 4096, 4096)
            if not ok or geo_img is None:
                with open(ocean_marker, "wb") as f_o:
                    pass
                return mx, my, mx_idx, my_idx, 0, 0.0, 0.0, 0, 0, 0, 0, {}

        if geo_img.max_elevation <= 0.01:
            with open(ocean_marker, "wb") as f_o:
                pass
            return mx, my, mx_idx, my_idx, 0, 0.0, 0.0, 0, 0, 0, 0, {}

        packer = HierarchicalTerrainPacker(geo_img, base_tile_size=64.0)
        max_mc_lod = 7
        mc_chunk_tiles = {}
        total_evaluated = 0

        for lod in range(max_mc_lod + 1):
            tile_sz = 64.0 * (2 ** lod)
            factor = 2 ** (max_mc_lod - lod)
            mc_chunk_tiles[lod] = {}

            for dy in range(factor):
                ty = dy
                sample_n = (mc_min_n + ty * tile_sz) if offline_source_img is not None else (-mc_sz * 0.5 + ty * tile_sz)
                for dx in range(factor):
                    tx = dx
                    sample_e = (mc_min_e + tx * tile_sz) if offline_source_img is not None else (-mc_sz * 0.5 + tx * tile_sz)
                    elevs = packer.sample_tile(sample_e, sample_n, tile_sz)
                    if HAS_NUMPY and isinstance(elevs, np.ndarray):
                        mc_chunk_tiles[lod][(tx, ty)] = elevs.reshape((HierarchicalTerrainPacker.SAMPLES_PER_TILE, HierarchicalTerrainPacker.SAMPLES_PER_TILE))
                    else:
                        mc_chunk_tiles[lod][(tx, ty)] = np.array(elevs, dtype=np.float32).reshape((HierarchicalTerrainPacker.SAMPLES_PER_TILE, HierarchicalTerrainPacker.SAMPLES_PER_TILE))
                    total_evaluated += 1

        pruned_in_mc = set()
        if args.prune_tolerance > 0 and HAS_NUMPY:
            factor0 = 2 ** max_mc_lod
            leaf_candidates = {(dx, dy) for dy in range(factor0) for dx in range(factor0)}

            for lod in range(1, max_mc_lod + 1):
                factor = 2 ** (max_mc_lod - lod)
                for dy in range(factor):
                    ty = dy
                    for dx in range(factor):
                        tx = dx
                        p_key = (tx, ty)
                        parent_elevs = mc_chunk_tiles[lod][p_key]

                        c00 = (2 * tx,     2 * ty)
                        c10 = (2 * tx + 1, 2 * ty)
                        c01 = (2 * tx,     2 * ty + 1)
                        c11 = (2 * tx + 1, 2 * ty + 1)
                        c_keys = [c00, c10, c01, c11]

                        if not all(k in leaf_candidates and (lod - 1, k[0], k[1]) not in pruned_in_mc for k in c_keys):
                            continue

                        quads = [(0, 0), (1, 0), (0, 1), (1, 1)]
                        can_prune = True
                        for (qx, qy), c_key in zip(quads, c_keys):
                            child_elevs = mc_chunk_tiles[lod - 1][c_key]
                            interp = bilinear_upsample_quadrant(parent_elevs, qx, qy)
                            err = float(np.max(np.abs(child_elevs - interp)))
                            if err > args.prune_tolerance:
                                can_prune = False
                                break

                        if can_prune:
                            for c_key in c_keys:
                                pruned_in_mc.add((lod - 1, c_key[0], c_key[1]))
                                leaf_candidates.remove(c_key)
                            leaf_candidates.add(p_key)

        num_kept = 0
        chunk_min_elev = 1e9
        chunk_max_elev = -1e9
        chunk_uncomp_bytes = 0
        chunk_comp_bytes = 0
        lod_counts = {l: 0 for l in range(max_mc_lod + 1)}

        index_entries = []
        payload_chunks = []
        curr_offset_in_chunk = 0

        for lod in range(max_mc_lod + 1):
            factor = 2 ** (max_mc_lod - lod)
            global_base_tx = mx_idx * factor
            global_base_ty = my_idx * factor

            for dy in range(factor):
                ty = dy
                global_ty = global_base_ty + ty
                for dx in range(factor):
                    tx = dx
                    global_tx = global_base_tx + tx

                    if (lod, tx, ty) in pruned_in_mc:
                        continue

                    elevs_2d = mc_chunk_tiles[lod][(tx, ty)]
                    min_h = float(np.min(elevs_2d))
                    max_h = float(np.max(elevs_2d))
                    chunk_min_elev = min(chunk_min_elev, min_h)
                    chunk_max_elev = max(chunk_max_elev, max_h)

                    if args.compression == "lerc" and HAS_LERC:
                        if args.format == "float32":
                            elevs_f32 = np.ascontiguousarray(elevs_2d, dtype=np.float32)
                            res, nbytes, compressed_payload = lerc.encode(elevs_f32, 1, False, None, args.lerc_tolerance, 1)
                            uncompressed_len = elevs_f32.nbytes
                            fmt_id = 2
                        else:
                            rng = max_h - min_h
                            if rng < 1e-4:
                                raw16 = np.zeros(elevs_2d.shape, dtype=np.uint16)
                                u16_tol = 1.0
                            else:
                                inv_rng = 65535.0 / rng
                                raw16 = np.clip((elevs_2d - min_h) * inv_rng + 0.5, 0, 65535).astype(np.uint16)
                                u16_tol = max(1.0, args.lerc_tolerance * inv_rng)
                            raw16 = np.ascontiguousarray(raw16)
                            res, nbytes, compressed_payload = lerc.encode(raw16, 1, False, None, u16_tol, 1)
                            uncompressed_len = raw16.nbytes
                            fmt_id = 3
                    else:
                        uncompressed_payload = elevs_2d.astype(np.float32).tobytes()
                        uncompressed_len = len(uncompressed_payload)
                        compressed_payload = zlib.compress(uncompressed_payload, level=6)
                        fmt_id = 1

                    m_key = pack_tile_key(lod, global_tx, global_ty)
                    comp_sz = len(compressed_payload)
                    entry = struct.pack("<QIIHHQIIff",
                                        m_key, global_tx, global_ty,
                                        lod, fmt_id,
                                        curr_offset_in_chunk,
                                        comp_sz, uncompressed_len,
                                        min_h, max_h)
                    index_entries.append(entry)
                    payload_chunks.append(compressed_payload)
                    curr_offset_in_chunk += comp_sz
                    chunk_uncomp_bytes += uncompressed_len
                    chunk_comp_bytes += comp_sz
                    lod_counts[lod] += 1
                    num_kept += 1

        tmp_path = chunk_bin + ".tmp"
        with open(tmp_path, "wb") as f_c:
            hdr = struct.pack("<4sIffIIII", b"MCHK", num_kept,
                              chunk_min_elev, chunk_max_elev,
                              len(pruned_in_mc), total_evaluated,
                              chunk_uncomp_bytes, chunk_comp_bytes)
            f_c.write(hdr)
            f_c.write(b"".join(index_entries))
            f_c.write(b"".join(payload_chunks))
        os.replace(tmp_path, chunk_bin)

        return mx, my, mx_idx, my_idx, num_kept, chunk_min_elev, chunk_max_elev, len(pruned_in_mc), total_evaluated, chunk_uncomp_bytes, chunk_comp_bytes, lod_counts
    except Exception as e:
        print(f"  [ERROR] Megachunk ({mx}, {my}) error: {e}")
        return mx, my, mx_idx, my_idx, 0, 0.0, 0.0, 0, 0, 0, 0, {}


def cmd_pipeline_megachunk(args):
    """
    Rolling Megachunk Pipeline:
    1) Iterates across 8192m x 8192m megachunks (LOD 7 roots = 128x128 of 64m LOD 0 tiles).
    2) For each megachunk:
       - Downloads raw bare-earth DEM compressed with LERC directly into memory from USGS 3DEP
         (or slices from input GeoTIFF).
       - Automatically skips pure ocean / nodata megachunks with 0 land relief.
       - Processes 0.5m hierarchical quadtree (LOD 0 to LOD 7).
       - Performs bottom-up adaptive quadtree pruning (residual error <= prune_tolerance).
       - Compresses kept nodes using LERC (error <= lerc_tolerance).
       - Saves compact megachunk cache binary to temp_megachunks/ (bounded memory < 500MB).
    3) Streams megachunk payloads into final container, sorts Morton index table via numpy,
       and removes temporary megachunks.
    """
    if args.compression == "lerc" and not HAS_LERC:
        print("[WARNING] LERC requested but lerc module not available. Falling back to Deflate.")
        args.compression = "deflate"

    min_lon, min_lat, max_lon, max_lat = args.bbox
    print(f"=== ForceEngine Rolling Megachunk Terrain Pipeline ===")
    print(f"  Target Bounding Box: [{min_lon:.5f}, {min_lat:.5f}] -> [{max_lon:.5f}, {max_lat:.5f}]")
    print(f"  Megachunk Physical Size: {args.megachunk_size:.0f}m x {args.megachunk_size:.0f}m (LOD 7 root)")
    print(f"  Base Tile (LOD 0): 64m x 64m ({HierarchicalTerrainPacker.SAMPLES_PER_TILE}x{HierarchicalTerrainPacker.SAMPLES_PER_TILE} samples, {args.resolution:.2f}m resolution)")
    print(f"  Compression: {args.compression.upper()} (max error: {args.lerc_tolerance*100:.1f}cm)")
    print(f"  Adaptive Pruning Tolerance: {args.prune_tolerance*100:.1f}cm")
    print(f"  Raw Data Retention: Ephemeral Stream (0 raw disk footprint)")

    # Compute global domain anchor and ENU frame
    center_lat = (min_lat + max_lat) * 0.5
    center_lon = (min_lon + max_lon) * 0.5
    center_ecef = WGS84.geodetic_to_ecef(center_lat, center_lon, 0.0)
    R_enu = WGS84.enu_rotation_matrix(center_lat, center_lon)

    sw_ecef = WGS84.geodetic_to_ecef(min_lat, min_lon, 0.0)
    ne_ecef = WGS84.geodetic_to_ecef(max_lat, max_lon, 0.0)
    e_sw, n_sw, _ = WGS84.ecef_to_enu(sw_ecef, center_ecef, R_enu)
    e_ne, n_ne, _ = WGS84.ecef_to_enu(ne_ecef, center_ecef, R_enu)

    min_e = min(e_sw, e_ne)
    max_e = max(e_sw, e_ne)
    min_n = min(n_sw, n_ne)
    max_n = max(n_sw, n_ne)

    mc_sz = float(args.megachunk_size)
    mc_x0_raw = int(math.floor(min_e / mc_sz))
    mc_x1_raw = int(math.ceil(max_e / mc_sz))
    mc_y0_raw = int(math.floor(min_n / mc_sz))
    mc_y1_raw = int(math.ceil(max_n / mc_sz))

    half_mc_x = max(abs(mc_x0_raw), abs(mc_x1_raw))
    half_mc_y = max(abs(mc_y0_raw), abs(mc_y1_raw))
    mc_x0 = -half_mc_x
    mc_x1 = half_mc_x
    mc_y0 = -half_mc_y
    mc_y1 = half_mc_y

    num_mc_x = 2 * half_mc_x
    num_mc_y = 2 * half_mc_y
    total_megachunks = num_mc_x * num_mc_y

    tiles_per_mc_lod0 = int(mc_sz / 64.0) # 128
    tiles_x_lod0 = num_mc_x * tiles_per_mc_lod0
    tiles_y_lod0 = num_mc_y * tiles_per_mc_lod0

    start_e = mc_x0 * mc_sz
    start_n = mc_y0 * mc_sz

    print(f"  Megachunk Grid: {num_mc_x} x {num_mc_y} = {total_megachunks} megachunks (symmetric domain)")
    print(f"  Total Domain LOD 0 Grid: {tiles_x_lod0} x {tiles_y_lod0} leaf tiles")
    print(f"  ENU Origin Anchor: ({start_e:.1f}m, {start_n:.1f}m)")

    # Optional Pre-filtering from Road Network GeoJSON
    candidate_megachunks = None
    road_filter_path = getattr(args, "road_filter", None)
    if road_filter_path and os.path.exists(road_filter_path):
        print(f"[PRE-FILTER] Filtering megachunks using road corridors from {road_filter_path}...")
        road_mcs = set()
        with open(road_filter_path, "r") as f_rf:
            for line in f_rf:
                if '"coordinates":' not in line:
                    continue
                try:
                    feat = json.loads(line.strip().rstrip(','))
                    for pt in feat['geometry']['coordinates']:
                        p_ecef = WGS84.geodetic_to_ecef(pt[1], pt[0], 0.0)
                        e, n, _ = WGS84.ecef_to_enu(p_ecef, center_ecef, R_enu)
                        rmx = int(math.floor(e / mc_sz))
                        rmy = int(math.floor(n / mc_sz))
                        road_mcs.add((rmx, rmy))
                except Exception:
                    pass
        candidate_megachunks = set()
        for rmx, rmy in road_mcs:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    candidate_megachunks.add((rmx + dx, rmy + dy))
        print(f"  Identified {len(road_mcs)} megachunks with roads ({len(candidate_megachunks)} including 1-ring margin).")

    # Prepare temp directory
    temp_dir = os.path.join(args.out_dir, "temp_megachunks")
    os.makedirs(temp_dir, exist_ok=True)

    global_min_elev = 1e9
    global_max_elev = -1e9
    total_uncompressed_bytes = 0
    total_compressed_bytes = 0
    total_pruned_tiles = 0
    total_evaluated_tiles = 0
    total_tiles_count = 0
    lod_tile_counts = {l: 0 for l in range(8)}

    # Pre-load offline source GeoTIFF if provided
    offline_source_img = None
    if args.input and os.path.exists(args.input):
        print(f"\n[SOURCE] Reading pre-downloaded source GeoTIFF: {args.input}")
        offline_source_img = GeoTIFFImage.read_from_file(args.input)

    pad = 64.0
    tasks = []
    for my_idx in range(num_mc_y):
        my = mc_y0 + my_idx
        for mx_idx in range(num_mc_x):
            mx = mc_x0 + mx_idx
            if candidate_megachunks is not None and (mx, my) not in candidate_megachunks:
                continue
            tasks.append((mx, my, mx_idx, my_idx))

    total_tasks = len(tasks)
    print(f"  Active Megachunks to Fetch & Compile: {total_tasks}")

    workers = getattr(args, "workers", 4)
    print(f"  Executing pipeline with {workers} concurrent workers...\n")

    completed_count = 0
    skipped_ocean_count = 0

    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
        future_to_task = {
            executor.submit(process_single_megachunk, mx, my, mx_idx, my_idx,
                            mc_sz, pad, center_ecef, R_enu, args, offline_source_img, temp_dir): (mx, my)
            for mx, my, mx_idx, my_idx in tasks
        }

        for future in concurrent.futures.as_completed(future_to_task):
            completed_count += 1
            mx, my, mx_idx, my_idx, num_t, chunk_min_elev, chunk_max_elev, pruned_cnt, eval_cnt, unc_b, cmp_b, lod_cnts = future.result()
            total_evaluated_tiles += eval_cnt
            total_pruned_tiles += pruned_cnt

            if num_t == 0:
                skipped_ocean_count += 1
                print(f"  [{completed_count}/{total_tasks}] Megachunk ({mx}, {my}): [SKIPPED Ocean/Nodata] (0 land relief)")
                continue

            global_min_elev = min(global_min_elev, chunk_min_elev)
            global_max_elev = max(global_max_elev, chunk_max_elev)
            total_uncompressed_bytes += unc_b
            total_compressed_bytes += cmp_b
            total_tiles_count += num_t
            for l, c in lod_cnts.items():
                lod_tile_counts[l] += c

            print(f"  [{completed_count}/{total_tasks}] Megachunk ({mx}, {my}): Kept {num_t:,} tiles ({pruned_cnt:,} pruned). Total retained: {total_tiles_count:,}")

    # Step 4: Final Container Assembly
    output_pack = args.out_pack or os.path.join(args.out_dir, f"{args.region_name}.fterrainpack")
    print(f"\n=== Assembling Master .fterrainpack Container ===")
    print(f"  Target File: {output_pack}")
    print(f"  Total Retained Tiles: {total_tiles_count:,}")
    print(f"  Total Pruned Tiles: {total_pruned_tiles:,} ({total_pruned_tiles / max(1, total_evaluated_tiles) * 100:.1f}%)")
    print(f"  Uncompressed Size: {total_uncompressed_bytes / (1024*1024):.2f} MB")
    print(f"  Compressed Size: {total_compressed_bytes / (1024*1024):.2f} MB ({total_compressed_bytes / max(1, total_uncompressed_bytes) * 100:.1f}%)")

    all_indices = np.empty(total_tiles_count, dtype=INDEX_DTYPE)
    curr_idx = 0
    curr_dst_payload_offset = HierarchicalTerrainPacker.PACK_HEADER_SIZE # 192 bytes

    os.makedirs(os.path.dirname(os.path.abspath(output_pack)), exist_ok=True)
    with open(output_pack, "wb") as f_dst:
        # 1. Header placeholder (192 bytes)
        f_dst.write(b'\x00' * HierarchicalTerrainPacker.PACK_HEADER_SIZE)

        # 2. Sequentially stream payload from each megachunk and collect index entries
        print(f"  Streaming megachunk payloads into container...")
        for mx, my, mx_idx, my_idx in tasks:
            chunk_bin = os.path.join(temp_dir, f"mc_{mx}_{my}.bin")
            if not os.path.exists(chunk_bin):
                continue

            with open(chunk_bin, "rb") as f_src:
                hdr_bytes = f_src.read(32)
                magic, num_mc_tiles, _, _, _, _, _, comp_sz = struct.unpack("<4sIffIIII", hdr_bytes)
                if num_mc_tiles == 0:
                    continue

                idx_bytes = f_src.read(num_mc_tiles * 44)
                mc_idx_view = np.frombuffer(idx_bytes, dtype=INDEX_DTYPE)
                all_indices[curr_idx : curr_idx + num_mc_tiles] = mc_idx_view
                all_indices['byteOffset'][curr_idx : curr_idx + num_mc_tiles] += curr_dst_payload_offset
                curr_idx += num_mc_tiles

                # Stream payload in 4MB chunks
                bytes_left = comp_sz
                while bytes_left > 0:
                    chunk_to_read = min(bytes_left, 4 * 1024 * 1024)
                    buf = f_src.read(chunk_to_read)
                    if not buf:
                        break
                    f_dst.write(buf)
                    bytes_left -= len(buf)
                curr_dst_payload_offset = f_dst.tell()

            # Delete chunk_bin immediately to free disk space
            try:
                os.remove(chunk_bin)
            except OSError:
                pass

        assert curr_idx == total_tiles_count, f"Index mismatch: {curr_idx} != {total_tiles_count}"
        index_table_offset = f_dst.tell()

        # 3. Sort index table by Morton tileKey
        print(f"  Sorting {total_tiles_count:,} index entries by Morton Z-order curve...")
        all_indices.sort(order='tileKey')

        # 4. Write sorted index table to destination
        print(f"  Writing index table ({all_indices.nbytes / (1024*1024):.2f} MB)...")
        f_dst.write(all_indices.tobytes())

        # 5. Write finalized 192-byte header at offset 0
        print("  Finalizing pack container header...")
        flags = (1 if args.format == "uint16" else 0) | (4 if args.compression == "lerc" else 2)
        enu_flat = [float(val) for row in R_enu for val in row]
        hdr_full = bytearray(HierarchicalTerrainPacker.PACK_HEADER_SIZE)
        struct.pack_into("<4sIfIf5IQQff3d2d9ffI44x", hdr_full, 0,
                         b"FTPK", 2,
                         64.0, # baseTileSize = 64m
                         HierarchicalTerrainPacker.SAMPLES_PER_TILE, # 129
                         args.resolution,
                         0, 7, # LOD 0..7
                         total_tiles_count,
                         tiles_x_lod0, tiles_y_lod0,
                         index_table_offset,
                         HierarchicalTerrainPacker.PACK_HEADER_SIZE, # dataOffset = 192
                         float(global_min_elev), float(global_max_elev),
                         center_ecef[0], center_ecef[1], center_ecef[2],
                         center_lat, center_lon,
                         *enu_flat,
                         WGS84.MEAN_RADIUS, flags)
        assert len(hdr_full) == 192
        f_dst.seek(0)
        f_dst.write(hdr_full)

    # Clean up temp_dir (ocean markers etc)
    for fname in os.listdir(temp_dir):
        try:
            os.remove(os.path.join(temp_dir, fname))
        except OSError:
            pass
    try:
        os.rmdir(temp_dir)
    except OSError:
        pass

    # Write Manifest JSON
    summary = {
        "format": "ForceEngine-TerrainPack",
        "version": 2,
        "container_file": os.path.basename(output_pack),
        "base_tile_size_meters": 64.0,
        "samples_per_tile": HierarchicalTerrainPacker.SAMPLES_PER_TILE,
        "base_resolution_meters": args.resolution,
        "lod_range": [0, 7],
        "megachunk_size_meters": mc_sz,
        "megachunk_grid": [num_mc_x, num_mc_y],
        "total_megachunks": total_megachunks,
        "compression": args.compression.upper(),
        "format_type": args.format,
        "prune_tolerance_meters": args.prune_tolerance,
        "lerc_tolerance_meters": args.lerc_tolerance,
        "pruned_tiles": total_pruned_tiles,
        "total_tiles": total_tiles_count,
        "tiles_per_lod": lod_tile_counts,
        "file_size_bytes": os.path.getsize(output_pack),
        "uncompressed_bytes": total_uncompressed_bytes,
        "compression_ratio": round(total_uncompressed_bytes / max(1, total_compressed_bytes), 2),
        "geodetic_center": { "lat": center_lat, "lon": center_lon },
        "ecef_origin": list(center_ecef),
        "elevation_range": [global_min_elev, global_max_elev]
    }
    meta_file = os.path.splitext(output_pack)[0] + "_manifest.json"
    with open(meta_file, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n[SUCCESS] Rolling Megachunk Pipeline Completed Successfully!")
    print(f"  Container: {output_pack} ({os.path.getsize(output_pack)/(1024*1024):.2f} MB on disk)")
    print(f"  Manifest: {meta_file}")
    return summary


def cmd_info(args):
    filepath = args.file
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)

    if filepath.endswith((".fterrainpack", ".fterrain")):
        with open(filepath, "rb") as f:
            hdr_bytes = f.read(192)
            if len(hdr_bytes) < 192:
                print("Invalid .fterrainpack: file too small")
                return

            magic = hdr_bytes[:4].decode('ascii', errors='ignore')
            version, base_tile_sz, samples_tile, base_res, min_lod, max_lod, tile_count, r0, r1 = struct.unpack_from("<IfIf5I", hdr_bytes, 4)
            idx_off, data_off = struct.unpack_from("<QQ", hdr_bytes, 40)
            min_e, max_h = struct.unpack_from("<ff", hdr_bytes, 56)
            ecef_x, ecef_y, ecef_z = struct.unpack_from("<3d", hdr_bytes, 64)
            lat, lon = struct.unpack_from("<2d", hdr_bytes, 88)

            print(f"=== .fterrainpack Container: {os.path.basename(filepath)} ===")
            print(f"  Magic: {magic}, Version: {version}")
            print(f"  Base Tile Size (LOD 0): {base_tile_sz:.1f}m x {base_tile_sz:.1f}m")
            print(f"  Samples Per Tile: {samples_tile} x {samples_tile} ({base_res:.2f}m resolution)")
            print(f"  LOD Pyramid: Levels {min_lod} through {max_lod}")
            print(f"  Road Chunk Integration: LOD 3 = {base_tile_sz * 8:.0f}m (exactly 1 Road Chunk = 8x8 LOD 0 tiles)")
            print(f"  Total Contained Tiles: {tile_count:,}")
            print(f"  Elevation Bounds: [{min_e:.2f}m, {max_h:.2f}m]")
            print(f"  Origin (WGS84): Lat {lat:.6f}°, Lon {lon:.6f}°")
            print(f"  Origin (ECEF): ({ecef_x:,.1f}, {ecef_y:,.1f}, {ecef_z:,.1f}) meters")
            print(f"  Index Table Offset: {idx_off:,} bytes ({tile_count * 44:,} bytes table)")
            print(f"  Data Blob Offset: {data_off:,} bytes")
            print(f"  Container File Size: {os.path.getsize(filepath):,} bytes on disk")

    elif filepath.endswith((".tif", ".tiff")):
        img = GeoTIFFImage.read_from_file(filepath)
        print(f"=== GeoTIFF DEM Info: {os.path.basename(filepath)} ===")
        print(f"  Dimensions: {img.width} x {img.height} pixels")
        print(f"  Bounds: [{img.min_lon:.6f}, {img.min_lat:.6f}] -> [{img.max_lon:.6f}, {img.max_lat:.6f}]")
        print(f"  Elevations: min = {img.min_elevation:.2f} m, max = {img.max_elevation:.2f} m")

    elif filepath.endswith(".json"):
        with open(filepath, "r") as f:
            print(f.read())


def cmd_pdal_config(args):
    pipeline_json = USGS3DEPClient.generate_pdal_pipeline_json(
        args.ept_url or USGS3DEPClient.NJ_LIDAR_EPT_URL,
        args.bounds[0], args.bounds[1], args.bounds[2], args.bounds[3],
        args.resolution, args.out_tif
    )
    with open(args.out_pipeline, "w") as f:
        f.write(pipeline_json)
    print(f"Generated PDAL 0.5m LiDAR pipeline configuration -> {args.out_pipeline}")
    print(f"Run via: pdal pipeline {args.out_pipeline}")


# ==============================================================================
# 6. MAIN CLI ENTRY POINT
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="ForceEngine Hierarchical Quadtree Terrain Pipeline (USGS 3DEP, 64m Base Tiles, 129x129, Single Pack Container)"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    PRESETS = {
        "jughandle": (-74.635, 40.565, -74.605, 40.595),
        "somerset_4k": (-74.640, 40.560, -74.600, 40.600),
        "somerset_05m": (-74.640, 40.5545, -74.593, 40.6015),
        "somerset_8k": (-74.670, 40.540, -74.570, 40.620),
        "nj_65k": (-75.007, 40.284, -74.231, 40.874),
        "new_jersey": (-75.60, 38.90, -73.85, 41.40),
    }

    # FETCH
    p_fetch = subparsers.add_parser("fetch", help="Fetch DEM from USGS 3DEP")
    p_fetch.add_argument("--preset", choices=PRESETS.keys(), default="jughandle")
    p_fetch.add_argument("--bbox", type=float, nargs=4)
    p_fetch.add_argument("--resolution", type=float, default=0.5,
                         help="Resolution in meters per pixel (default: 0.5m)")
    p_fetch.add_argument("--out-dir", default="jughandle/terrain_data_source")
    p_fetch.add_argument("--out-filename", default=None)

    # PREPROCESS
    p_prep = subparsers.add_parser("preprocess", help="Compile GeoTIFF into a single .fterrainpack container")
    p_prep.add_argument("--input", default="jughandle/terrain_data_source/dem_jughandle.tif")
    p_prep.add_argument("--out-dir", default="jughandle")
    p_prep.add_argument("--out-pack", default=None)
    p_prep.add_argument("--region-name", default="jughandle")
    p_prep.add_argument("--base-tile-size", type=float, default=None,
                        help="Base tile dimension in meters at LOD 0 (e.g. 64.0 for local, 4096.0 for state)")
    p_prep.add_argument("--max-lod", type=int, default=None,
                        help="Maximum LOD level (auto-calculated up to root if omitted)")
    p_prep.add_argument("--compression", choices=["lerc", "deflate"], default="lerc",
                        help="Tile compression algorithm (lerc: Limited Error Raster Compression, deflate: zlib)")
    p_prep.add_argument("--format", choices=["float32", "uint16"], default="float32",
                        help="Tile sample format (float32: native IEEE float heights, uint16: normalized)")
    p_prep.add_argument("--prune-tolerance", type=float, default=0.02,
                        help="Adaptive quadtree pruning tolerance in meters (default: 0.02m / 2cm)")
    p_prep.add_argument("--lerc-tolerance", type=float, default=0.015,
                        help="Maximum compression error for LERC in meters (default: 0.015m / 1.5cm)")

    # PIPELINE
    p_pipe = subparsers.add_parser("pipeline", help="Fetch and compile quadtree pack end-to-end")
    p_pipe.add_argument("--preset", choices=PRESETS.keys(), default="jughandle")
    p_pipe.add_argument("--bbox", type=float, nargs=4)
    p_pipe.add_argument("--resolution", type=float, default=0.5)
    p_pipe.add_argument("--base-tile-size", type=float, default=None,
                        help="Base tile dimension in meters at LOD 0")
    p_pipe.add_argument("--raw-dir", default="jughandle/terrain_data_source")
    p_pipe.add_argument("--out-dir", default="jughandle")
    p_pipe.add_argument("--region-name", default="jughandle")
    p_pipe.add_argument("--max-lod", type=int, default=None)
    p_pipe.add_argument("--compression", choices=["lerc", "deflate"], default="lerc")
    p_pipe.add_argument("--format", choices=["float32", "uint16"], default="float32")
    p_pipe.add_argument("--prune-tolerance", type=float, default=0.02)
    p_pipe.add_argument("--lerc-tolerance", type=float, default=0.015)

    # PIPELINE-MEGACHUNK (Rolling Download -> Process -> Delete Raw Data)
    p_mega = subparsers.add_parser("pipeline-megachunk",
                                   help="Rolling megachunk pipeline: Download chunk -> 0.5m quadtree & adaptive pruning & LERC -> Delete raw data")
    p_mega.add_argument("--preset", choices=PRESETS.keys(), default="new_jersey")
    p_mega.add_argument("--bbox", type=float, nargs=4, default=None)
    p_mega.add_argument("--megachunk-size", type=float, default=8192.0,
                        help="Megachunk physical width in meters (default: 8192m = LOD 7 root)")
    p_mega.add_argument("--resolution", type=float, default=0.5,
                        help="Base resolution in meters per sample (default: 0.5m)")
    p_mega.add_argument("--compression", choices=["lerc", "deflate"], default="lerc")
    p_mega.add_argument("--format", choices=["float32", "uint16"], default="float32")
    p_mega.add_argument("--prune-tolerance", type=float, default=0.02,
                        help="Adaptive quadtree pruning tolerance in meters (default: 0.02m / 2cm)")
    p_mega.add_argument("--lerc-tolerance", type=float, default=0.015,
                        help="LERC maximum error tolerance in meters (default: 0.015m / 1.5cm)")
    p_mega.add_argument("--keep-raw", action="store_true", default=False,
                        help="Keep raw downloaded GeoTIFFs on disk instead of deleting them immediately")
    p_mega.add_argument("--input", default=None,
                        help="Optional pre-existing source GeoTIFF to slice from instead of downloading")
    p_mega.add_argument("--out-dir", default="jughandle")
    p_mega.add_argument("--out-pack", default=None)
    p_mega.add_argument("--region-name", default="new_jersey_05m")
    p_mega.add_argument("--workers", type=int, default=4,
                        help="Number of concurrent worker threads (default: 4)")
    p_mega.add_argument("--road-filter", default=None,
                        help="Path to GeoJSON road network to pre-filter candidate megachunks")

    # INFO
    p_info = subparsers.add_parser("info", help="Inspect .fterrainpack container or GeoTIFF metadata")
    p_info.add_argument("file", help="Path to .fterrainpack, .tif, or manifest JSON")

    # PDAL-CONFIG
    p_pdal = subparsers.add_parser("pdal-config", help="Generate PDAL 0.5m LiDAR pipeline JSON")
    p_pdal.add_argument("--ept-url", default=USGS3DEPClient.NJ_LIDAR_EPT_URL)
    p_pdal.add_argument("--bounds", type=float, nargs=4, default=[-8310000, 4940000, -8305000, 4945000])
    p_pdal.add_argument("--resolution", type=float, default=0.5)
    p_pdal.add_argument("--out-tif", default="jughandle/terrain_data_source/nj_05m_ground.tif")
    p_pdal.add_argument("--out-pipeline", default="jughandle/terrain_data_source/pdal_pipeline.json")

    args = parser.parse_args()

    if args.command == "fetch":
        if not args.bbox:
            args.bbox = PRESETS[args.preset]
        if not args.out_filename:
            args.out_filename = f"dem_{args.preset}_{args.resolution}m.tif"
        cmd_fetch(args)

    elif args.command == "preprocess":
        cmd_preprocess(args)

    elif args.command == "pipeline":
        if not args.bbox:
            args.bbox = PRESETS[args.preset]
        if args.region_name == "jughandle" and args.preset != "jughandle":
            args.region_name = args.preset
        args.out_dir = args.raw_dir
        args.out_filename = f"dem_{args.preset}_{args.resolution}m.tif"
        raw_tif = cmd_fetch(args)

        args.input = raw_tif
        args.out_dir = "jughandle"
        cmd_preprocess(args, raw_tif)

    elif args.command == "pipeline-megachunk":
        if not args.bbox:
            args.bbox = PRESETS[args.preset]
        if args.region_name == "new_jersey_05m" and args.preset != "new_jersey":
            args.region_name = f"{args.preset}_05m"
        cmd_pipeline_megachunk(args)

    elif args.command == "info":
        cmd_info(args)

    elif args.command == "pdal-config":
        cmd_pdal_config(args)


if __name__ == "__main__":
    main()
