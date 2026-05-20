"""
Generate a synthetic rock-slope point cloud as a plain-text .xyzn file
(x y z nx ny nz per line). No third-party libraries required.

Both the web app (drag-and-drop) and the desktop app can read this file
directly, with per-point normals already included (no normal estimation
needed). Run:  python generate_sample_xyzn.py

The model contains a main slope face plus three joint sets chosen so that
the FAA analysis, run with the suggested parameters below, produces clear
sliding, toppling and wedge results.
"""

import math, random

random.seed(42)

# Slope geometry the joint sets are designed against
SLOPE_DIP, SLOPE_DIR = 70.0, 140.0


def plane_basis(dip, dip_dir):
    """Return (normal, strike_vec, dip_vec) for a plane dipping `dip` toward
    azimuth `dip_dir`. All unit vectors; z is up, x=East, y=North."""
    d, a = math.radians(dip), math.radians(dip_dir)
    n = (math.sin(d) * math.sin(a), math.sin(d) * math.cos(a), math.cos(d))
    # horizontal strike vector (perpendicular to the dip direction)
    u = (math.cos(a), -math.sin(a), 0.0)
    # down-dip in-plane vector = n x u
    v = (n[1] * u[2] - n[2] * u[1],
         n[2] * u[0] - n[0] * u[2],
         n[0] * u[1] - n[1] * u[0])
    return n, u, v


def jitter(val, s):
    return val + random.uniform(-s, s)


def make_patch(n, dip, dip_dir, cx, cy, cz, half_w, half_h, normal_noise):
    """A genuinely planar patch: points lie on the plane (small off-plane
    roughness only), so the cloud looks like a real rock face."""
    (nx, ny, nz), (ux, uy, uz), (vx, vy, vz) = plane_basis(dip, dip_dir)
    rows = []
    for _ in range(n):
        a = random.uniform(-half_w, half_w)   # along strike
        b = random.uniform(-half_h, half_h)   # down dip
        w = random.gauss(0, 0.04)             # off-plane roughness
        x = cx + a * ux + b * vx + w * nx
        y = cy + a * uy + b * vy + w * ny
        z = cz + a * uz + b * vz + w * nz
        ax, ay, az = (jitter(nx, normal_noise),
                      jitter(ny, normal_noise),
                      jitter(nz, normal_noise))
        m = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        rows.append((x, y, z, ax / m, ay / m, az / m))
    return rows


def main():
    rows = []
    # Cut face (background) — steeper than the design slope (80/140) so the
    # face itself neither daylights (sliding) nor dips into the slope
    # (toppling); it stays "stable" grey and only the joint sets are flagged.
    rows += make_patch(2600, 80, SLOPE_DIR, 0, 0, 8, 10.0, 9.0, 0.05)
    # J1 sliding set: joints dipping the same way as the slope but flatter
    # than the face (45/140) — steeper than friction, so they daylight & slide.
    for cx, cy, cz in [(-5, -3, 9), (3, 1, 7), (-1, 4, 11)]:
        rows += make_patch(300, 45, 140, cx, cy, cz, 2.2, 2.0, 0.04)
    # J2 toppling set: steep slabs dipping into the slope (85/320)
    for cx, cy, cz in [(-4, 0, 6), (2, -2, 9), (5, 2, 11)]:
        rows += make_patch(240, 85, 320, cx, cy, cz, 1.6, 2.4, 0.04)
    # Wedge sets J3 (45/110) and J4 (45/170): their line of intersection
    # plunges ~41 deg toward 140 and daylights on the slope. The two sets
    # share cluster centres so their points interleave under k-NN search.
    for cx, cy, cz in [(-3, 3, 9), (3, -1, 8)]:
        rows += make_patch(300, 45, 110, cx, cy, cz, 1.8, 1.8, 0.04)
        rows += make_patch(300, 45, 170, cx, cy, cz, 1.8, 1.8, 0.04)

    with open("sample_slope.xyzn", "w") as f:
        f.write("# x y z nx ny nz  — synthetic FAA test slope\n")
        for x, y, z, nx, ny, nz in rows:
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {nx:.4f} {ny:.4f} {nz:.4f}\n")

    print(f"Saved sample_slope.xyzn ({len(rows):,} points)")
    print("Suggested parameters:")
    print(f"  Slope Dip {SLOPE_DIP}  Dip Direction {SLOPE_DIR}")
    print("  Friction Angle 30,  Lateral (sliding) 20,  Lateral (toppling) 20")
    print("  Min angular diff (wedge) 30,  k-neighbours 16")


if __name__ == "__main__":
    main()
