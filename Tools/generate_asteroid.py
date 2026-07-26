#!/usr/bin/env python3
"""Generates Assets/Models/asteroid.glb — a displaced icosphere test asset.

Offline content tool (Tools module). Deterministic: same seed, same output.
Usage: python Tools/generate_asteroid.py [output.glb]
"""

import json
import math
import struct
import sys
from pathlib import Path

SEED = 1337
RADIUS = 2.0
SUBDIVISIONS = 3
NOISE_AMPLITUDE = 0.45
BASE_COLOR = None  # per-vertex colors omitted; material-free grey in engine


def hash01(x: int) -> float:
    """Deterministic integer hash to [0,1)."""
    x = (x ^ SEED) & 0xFFFFFFFF
    x = (x ^ (x >> 16)) * 0x45D9F3B & 0xFFFFFFFF
    x = (x ^ (x >> 16)) * 0x45D9F3B & 0xFFFFFFFF
    x = x ^ (x >> 16)
    return x / 0xFFFFFFFF


def value_noise(p, freq, salt):
    """Tri-linear value noise on a lattice, deterministic."""
    x, y, z = (c * freq + 100.0 for c in p)
    xi, yi, zi = int(math.floor(x)), int(math.floor(y)), int(math.floor(z))
    xf, yf, zf = x - xi, y - yi, z - zi
    s = lambda t: t * t * (3 - 2 * t)
    xf, yf, zf = s(xf), s(yf), s(zf)

    def lattice(ix, iy, iz):
        return hash01(ix * 374761393 + iy * 668265263 + iz * 2147483647 + salt * 973)

    def lerp(a, b, t):
        return a + (b - a) * t

    c00 = lerp(lattice(xi, yi, zi), lattice(xi + 1, yi, zi), xf)
    c10 = lerp(lattice(xi, yi + 1, zi), lattice(xi + 1, yi + 1, zi), xf)
    c01 = lerp(lattice(xi, yi, zi + 1), lattice(xi + 1, yi, zi + 1), xf)
    c11 = lerp(lattice(xi, yi + 1, zi + 1), lattice(xi + 1, yi + 1, zi + 1), xf)
    return lerp(lerp(c00, c10, yf), lerp(c01, c11, yf), zf)


def fbm(p):
    total, amplitude, freq = 0.0, 1.0, 0.9
    for octave in range(4):
        total += (value_noise(p, freq, octave) * 2.0 - 1.0) * amplitude
        amplitude *= 0.5
        freq *= 2.1
    return total


def icosphere(subdivisions):
    t = (1.0 + math.sqrt(5.0)) / 2.0
    verts = [
        (-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0),
        (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
        (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1),
    ]
    verts = [normalize(v) for v in verts]
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ]
    cache = {}

    def midpoint(a, b):
        key = (min(a, b), max(a, b))
        if key in cache:
            return cache[key]
        m = normalize(tuple((verts[a][i] + verts[b][i]) * 0.5 for i in range(3)))
        verts.append(m)
        cache[key] = len(verts) - 1
        return cache[key]

    for _ in range(subdivisions):
        new_faces = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    return verts, faces


def normalize(v):
    length = math.sqrt(sum(c * c for c in v)) or 1.0
    return tuple(c / length for c in v)


def main():
    out_path = Path(sys.argv[1] if len(sys.argv) > 1 else "Assets/Models/asteroid.glb")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    unit_verts, faces = icosphere(SUBDIVISIONS)

    # Displace along the unit direction with fBm noise.
    positions = []
    for v in unit_verts:
        displacement = RADIUS * (1.0 + NOISE_AMPLITUDE * fbm(v))
        positions.append(tuple(c * displacement for c in v))

    # Smooth normals: accumulate face normals (area-weighted via cross product).
    normals = [[0.0, 0.0, 0.0] for _ in positions]
    for a, b, c in faces:
        pa, pb, pc = positions[a], positions[b], positions[c]
        u = tuple(pb[i] - pa[i] for i in range(3))
        w = tuple(pc[i] - pa[i] for i in range(3))
        n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        for idx in (a, b, c):
            for i in range(3):
                normals[idx][i] += n[i]
    normals = [normalize(tuple(n)) for n in normals]

    # glTF winding is CCW from outside; icosphere faces above already are.
    indices = [i for face in faces for i in face]

    # ---- binary buffers -----------------------------------------------------
    pos_bytes = b"".join(struct.pack("<3f", *p) for p in positions)
    nrm_bytes = b"".join(struct.pack("<3f", *n) for n in normals)
    idx_bytes = b"".join(struct.pack("<I", i) for i in indices)
    while len(idx_bytes) % 4:
        idx_bytes += b"\x00"
    buffer = pos_bytes + nrm_bytes + idx_bytes

    mins = [min(p[i] for p in positions) for i in range(3)]
    maxs = [max(p[i] for p in positions) for i in range(3)]

    gltf = {
        "asset": {"version": "2.0", "generator": "StarWorks Tools/generate_asteroid.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": "Asteroid"}],
        "meshes": [{
            "primitives": [{
                "attributes": {"POSITION": 0, "NORMAL": 1},
                "indices": 2,
                "mode": 4,
            }],
            "name": "AsteroidMesh",
        }],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": len(positions),
             "type": "VEC3", "min": mins, "max": maxs},
            {"bufferView": 1, "componentType": 5126, "count": len(normals), "type": "VEC3"},
            {"bufferView": 2, "componentType": 5125, "count": len(indices), "type": "SCALAR"},
        ],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(pos_bytes), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_bytes), "byteLength": len(nrm_bytes),
             "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_bytes) + len(nrm_bytes),
             "byteLength": len(idx_bytes), "target": 34963},
        ],
        "buffers": [{"byteLength": len(buffer)}],
    }

    json_bytes = json.dumps(gltf, separators=(",", ":")).encode()
    while len(json_bytes) % 4:
        json_bytes += b" "

    header = struct.pack("<3I", 0x46546C67, 2, 12 + 8 + len(json_bytes) + 8 + len(buffer))
    json_chunk = struct.pack("<2I", len(json_bytes), 0x4E4F534A) + json_bytes
    bin_chunk = struct.pack("<2I", len(buffer), 0x004E4942) + buffer

    out_path.write_bytes(header + json_chunk + bin_chunk)
    print(f"Wrote {out_path} ({len(positions)} vertices, {len(faces)} triangles)")


if __name__ == "__main__":
    main()
