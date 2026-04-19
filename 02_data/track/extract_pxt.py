"""
extract_pxt.py — Extract GPS centerline from iRacing .pxt track-map files.

The .pxt file 'Nurburgring Combined Track.pxt' is an OLE2 Compound Document
with two streams:
  - Version      : uint32, schema version (= 6 here)
  - GPSMapStream : raw binary with a header + 112-byte repeating block

Reverse-engineered layout of GPSMapStream (little-endian):

  [header, variable length — starts at 0x00]
    0x00  double   reserved (looks like a bounding-box magic)
    0x08  uint32   title byte-length  (e.g. 42)
    0x0C  UTF-16LE track title        (e.g. 'Nurburgring Combined')
    ...   sector-record table         (variable — 4x int32 bbox + two
                                       length-prefixed UTF-16LE sector names
                                       + several uint32 indices per sector)

  [point array — starts at 0x1A6E in this file, 8393 x 112-byte blocks]
    Each 112-byte block holds ONE sample of the centerline plus left/right
    edges and extra metadata.  Field map inside each block:

      offset  type     meaning
      ------  -------  --------------------------------------------------
         0    double   edge1 latitude  [rad]
         8    double   edge1 longitude [rad]
        16    double   edge1 elevation [m MSL]
        24    double   edge2 latitude  [rad]
        32    double   edge2 longitude [rad]
        40    double   edge2 elevation [m MSL]
        48    double   centerline latitude  [rad]
        56    double   centerline longitude [rad]
        64    double   local X         [m]    (east, relative to first pt)
        72    double   local Y         [m]    (north, relative to first pt)
        80    double   reserved        (~ curvature or banking, small)
        88    double   cumulative distance [m]  (spacing is 3 m nominally)
        96    double   centerline elevation [m MSL]
       104    double   reserved        (~ heading or orientation, ~3.9 rad)

  [trailer, ~64 bytes, ignored]

Output: pxt_centerline.csv  — one row per native sample, columns
  idx, dist_m, x_m, y_m, elev_m, lat_deg, lon_deg,
  edge1_lat_deg, edge1_lon_deg, edge2_lat_deg, edge2_lon_deg

The MATLAB script `build_track_from_gps.m` consumes this CSV, resamples
to 1 m, and computes geometric curvature.

Requirements:  pip install olefile
Usage:         python extract_pxt.py
"""

import math
import struct
import csv
from pathlib import Path

import olefile


PXT_PATH = Path(__file__).parent / "Nurburgring Combined Track.pxt"
CSV_PATH = Path(__file__).parent / "pxt_centerline.csv"

# Offset at which the 112-byte point-array starts, determined by inspection
# of the sector-record table for this particular track.  If a different
# .pxt is used, this will need re-discovering — see `find_array_start()`.
ARRAY_OFFSET = 0x1A6E
BLOCK_SIZE   = 112

# Nürburgring GPS plausibility bounds (for block-validation while scanning)
LAT_MIN, LAT_MAX = 0.87, 0.89      # rad  (50.0-51.0 deg)
LON_MIN, LON_MAX = 0.118, 0.125    # rad  (6.75-7.18 deg)
ELEV_MIN, ELEV_MAX = 200.0, 900.0  # m MSL


def find_array_start(data, lat_min=LAT_MIN, lat_max=LAT_MAX,
                     lon_min=LON_MIN, lon_max=LON_MAX):
    """Walk through the stream and return the offset of the first block
    whose first (lat, lon) double pair lands in the expected range."""
    for i in range(0, len(data) - 16):
        (lat,) = struct.unpack_from("<d", data, i)
        if lat_min < lat < lat_max:
            (lon,) = struct.unpack_from("<d", data, i + 8)
            if lon_min < lon < lon_max:
                return i
    raise RuntimeError("No valid lat/lon pair found in stream")


def parse_block(data, off):
    """Return (edge1, edge2, cl_lat, cl_lon, x, y, dist, cl_elev) for one
    block.  Each edge is (lat, lon, elev)."""
    e1 = struct.unpack_from("<3d", data, off)
    e2 = struct.unpack_from("<3d", data, off + 24)
    cl_lat, cl_lon = struct.unpack_from("<2d", data, off + 48)
    _x, _y, _, dist = struct.unpack_from("<4d", data, off + 64)
    (cl_elev,) = struct.unpack_from("<d", data, off + 96)
    return e1, e2, cl_lat, cl_lon, _x, _y, dist, cl_elev


def block_is_valid(e1, e2, cl_lat, cl_lon, cl_elev):
    def lat_ok(v): return LAT_MIN < v < LAT_MAX
    def lon_ok(v): return LON_MIN < v < LON_MAX
    def elev_ok(v): return ELEV_MIN < v < ELEV_MAX
    return (lat_ok(e1[0]) and lon_ok(e1[1]) and elev_ok(e1[2])
            and lat_ok(e2[0]) and lon_ok(e2[1]) and elev_ok(e2[2])
            and lat_ok(cl_lat) and lon_ok(cl_lon) and elev_ok(cl_elev))


def main():
    print(f"Reading {PXT_PATH.name} ...")
    f = olefile.OleFileIO(str(PXT_PATH))

    with f.openstream("Version") as v:
        version = struct.unpack("<I", v.read())[0]
    print(f"  Schema version: {version}")

    with f.openstream("GPSMapStream") as s:
        data = s.read()
    print(f"  GPSMapStream: {len(data):,} bytes")

    # Confirm / locate the point-array start
    found = find_array_start(data)
    if found != ARRAY_OFFSET:
        print(f"  WARNING: expected array at {ARRAY_OFFSET:#x}, "
              f"found plausible data at {found:#x}")
    array_off = found
    print(f"  Point array starts at {array_off:#x}")

    # Walk the array until we hit a block that fails sanity checks
    rows, off = [], array_off
    R2D = 180.0 / math.pi
    while off + BLOCK_SIZE <= len(data):
        e1, e2, cl_lat, cl_lon, x, y, dist, cl_elev = parse_block(data, off)
        if not block_is_valid(e1, e2, cl_lat, cl_lon, cl_elev):
            break
        rows.append((
            len(rows), dist, x, y, cl_elev,
            cl_lat * R2D, cl_lon * R2D,
            e1[0] * R2D, e1[1] * R2D,
            e2[0] * R2D, e2[1] * R2D,
        ))
        off += BLOCK_SIZE

    print(f"  Extracted {len(rows):,} blocks.")
    print(f"  Consumed {off - array_off:,} bytes, "
          f"{len(data) - off} bytes trailing.")
    print(f"  Track length: {rows[-1][1]:.1f} m")

    with CSV_PATH.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([
            "idx", "dist_m", "x_m", "y_m", "elev_m",
            "lat_deg", "lon_deg",
            "edge1_lat_deg", "edge1_lon_deg",
            "edge2_lat_deg", "edge2_lon_deg",
        ])
        for r in rows:
            w.writerow([
                r[0], f"{r[1]:.4f}", f"{r[2]:.4f}", f"{r[3]:.4f}",
                f"{r[4]:.3f}",
                f"{r[5]:.8f}", f"{r[6]:.8f}",
                f"{r[7]:.8f}", f"{r[8]:.8f}",
                f"{r[9]:.8f}", f"{r[10]:.8f}",
            ])
    print(f"Wrote {CSV_PATH}")


if __name__ == "__main__":
    main()
