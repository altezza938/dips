"""
Generate a REALISTIC synthetic rock-slope point cloud as a coloured ASCII .ply
file (x y z  nx ny nz  red green blue). No third-party libraries required.

Unlike sample_slope.xyzn (idealised flat patches), this model is built to look
like a real terrestrial-laser / photogrammetry scan of a vegetated cliff:

  * a large, gently undulating rock face (grey-brown rock colour),
  * scattered GREEN VEGETATION clumps with near-random normals (these are the
    noise that the "Amalgamate into facets" option is designed to discard),
  * surface roughness on every patch,
  * three structural joint sets embedded as fresher-rock outcrops that drive
    the FAA analysis: planar SLIDING, flexural TOPPLING and a WEDGE.

Drag sample_slope.ply onto the web app (the PLY loader keeps both the normals
and the colours, so the 3-D view shows natural rock + green vegetation). Turn
ON "Amalgamate into facets" and click Run All — the vegetation is rejected as
small noisy regions and the joint facets are flagged.

Run:  python generate_sample_ply.py
"""

import math
import random

random.seed(7)

# Slope geometry the joint sets are designed against.
SLOPE_DIP, SLOPE_DIR = 70.0, 140.0


def plane_basis(dip, dip_dir):
    """(normal, strike_vec, dip_vec) for a plane dipping `dip` toward `dip_dir`.
    Unit vectors; z up, x=East, y=North."""
    d, a = math.radians(dip), math.radians(dip_dir)
    n = (math.sin(d) * math.sin(a), math.sin(d) * math.cos(a), math.cos(d))
    u = (math.cos(a), -math.sin(a), 0.0)                 # horizontal strike
    v = (n[1] * u[2] - n[2] * u[1],                      # down-dip = n x u
         n[2] * u[0] - n[0] * u[2],
         n[0] * u[1] - n[1] * u[0])
    return n, u, v


def norm3(x, y, z):
    m = math.sqrt(x * x + y * y + z * z) or 1.0
    return x / m, y / m, z / m


def rock_colour(base=(112, 103, 92), spread=22):
    """Grey-brown rock with per-point variation (RGB 0-255)."""
    r = base[0] + random.uniform(-spread, spread)
    g = base[1] + random.uniform(-spread, spread)
    b = base[2] + random.uniform(-spread, spread)
    # slight warm streaking (iron staining)
    if random.random() < 0.15:
        r += random.uniform(0, 30); g += random.uniform(0, 12)
    return _clamp(r), _clamp(g), _clamp(b)


def veg_colour():
    """Green vegetation with variation (RGB 0-255)."""
    g = random.uniform(95, 165)
    r = g * random.uniform(0.35, 0.62)
    b = g * random.uniform(0.28, 0.5)
    return _clamp(r), _clamp(g), _clamp(b)


def _clamp(v):
    return int(max(0, min(255, round(v))))


def make_rock_patch(n, dip, dip_dir, cx, cy, cz, half_w, half_h,
                    roughness=0.05, normal_noise=0.05, colour=None, wavy=0.0):
    """A rock patch lying on a plane, with optional large-scale undulation
    (`wavy`) so the main face looks naturally curved rather than flat."""
    (nx, ny, nz), (ux, uy, uz), (vx, vy, vz) = plane_basis(dip, dip_dir)
    rows = []
    for _ in range(n):
        a = random.uniform(-half_w, half_w)        # along strike
        b = random.uniform(-half_h, half_h)        # down dip
        # gentle undulation of the surface (keeps mean orientation intact)
        w = random.gauss(0, roughness)
        if wavy:
            w += wavy * (math.sin(a * 0.45) * math.cos(b * 0.55))
        x = cx + a * ux + b * vx + w * nx
        y = cy + a * uy + b * vy + w * ny
        z = cz + a * uz + b * vz + w * nz
        an, bn, cn = norm3(nx + random.uniform(-normal_noise, normal_noise),
                           ny + random.uniform(-normal_noise, normal_noise),
                           nz + random.uniform(-normal_noise, normal_noise))
        cr, cg, cb = colour() if colour else rock_colour()
        rows.append((x, y, z, an, bn, cn, cr, cg, cb))
    return rows


def make_vegetation(n, cx, cy, cz, radius):
    """A bushy clump: points scattered in a blob with near-random normals
    (high roughness) and green colour — i.e. exactly the kind of noise the
    facet amalgamation step is meant to throw away."""
    rows = []
    for _ in range(n):
        # blob offset
        dx = random.gauss(0, radius)
        dy = random.gauss(0, radius)
        dz = random.gauss(0, radius * 0.8)
        nx, ny, nz = norm3(random.uniform(-1, 1),
                           random.uniform(-1, 1),
                           random.uniform(-1, 1))
        cr, cg, cb = veg_colour()
        rows.append((cx + dx, cy + dy, cz + dz, nx, ny, nz, cr, cg, cb))
    return rows


def main():
    rows = []

    # ── Main cut face ────────────────────────────────────────────────────────
    # Steeper than the design slope (80/140) so the face itself neither
    # daylights (sliding) nor dips back into the slope (toppling): it stays the
    # stable grey-brown background. Large `wavy` term gives natural curvature.
    rows += make_rock_patch(4200, 80, SLOPE_DIR, 0, 0, 8, 11.0, 9.5,
                            roughness=0.10, normal_noise=0.06, wavy=0.55)

    # ── J1 — planar SLIDING set (45/140) ──────────────────────────────────────
    # Dips the same way as the slope but flatter than the face, and steeper than
    # friction (30°): it daylights and slides. Fresher, lighter rock.
    for cx, cy, cz in [(-5, -3, 9), (3, 1, 7), (-1, 4, 11)]:
        rows += make_rock_patch(320, 45, 140, cx, cy, cz, 2.2, 2.0,
                                colour=lambda: rock_colour((140, 132, 120), 16))

    # ── J2 — flexural TOPPLING set (85/320) ───────────────────────────────────
    # Steep slabs dipping back into the slope.
    for cx, cy, cz in [(-4, 0, 6), (2, -2, 9), (5, 2, 11)]:
        rows += make_rock_patch(260, 85, 320, cx, cy, cz, 1.6, 2.4,
                                colour=lambda: rock_colour((128, 120, 110), 16))

    # ── J3 (45/110) + J4 (45/170) — WEDGE pair ────────────────────────────────
    # Their line of intersection plunges ~41° toward 140 and daylights on the
    # slope. The two sets share cluster centres so points interleave under k-NN.
    for cx, cy, cz in [(-3, 3, 9), (3, -1, 8)]:
        rows += make_rock_patch(320, 45, 110, cx, cy, cz, 1.8, 1.8,
                                colour=lambda: rock_colour((146, 138, 126), 16))
        rows += make_rock_patch(320, 45, 170, cx, cy, cz, 1.8, 1.8,
                                colour=lambda: rock_colour((146, 138, 126), 16))

    # ── Vegetation clumps scattered over the face and toe ─────────────────────
    veg_spots = [(-8, 5, 12, 1.1), (6, 4, 12, 0.9), (-2, -4, 5, 1.0),
                 (8, -3, 6, 0.8), (0, 1, 9, 0.7), (-6, -1, 7, 0.9),
                 (4, 6, 13, 0.8), (-9, -4, 4, 1.0)]
    for cx, cy, cz, r in veg_spots:
        rows += make_vegetation(180, cx, cy, cz, r)

    random.shuffle(rows)   # interleave so the file isn't ordered by region

    with open("sample_slope.ply", "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment synthetic vegetated rock slope — FAA test model\n")
        f.write(f"element vertex {len(rows)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property float nx\nproperty float ny\nproperty float nz\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for x, y, z, nx, ny, nz, r, g, b in rows:
            f.write(f"{x:.4f} {y:.4f} {z:.4f} "
                    f"{nx:.4f} {ny:.4f} {nz:.4f} {r} {g} {b}\n")

    print(f"Saved sample_slope.ply ({len(rows):,} points)")
    print("Drag it onto the web app, turn ON 'Amalgamate into facets', Run All.")
    print("Suggested parameters:")
    print(f"  Slope Dip {SLOPE_DIP}  Dip Direction {SLOPE_DIR}")
    print("  Friction Angle 30,  Lateral (sliding) 20,  Lateral (toppling) 20")
    print("  Min angular diff (wedge) 30,  k-neighbours 16")


if __name__ == "__main__":
    main()
