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


def plane_normal(dip, dip_dir):
    """Upward unit normal of a plane dipping `dip` toward azimuth `dip_dir`.
    Horizontal component points toward the dip direction; z is up."""
    d, a = math.radians(dip), math.radians(dip_dir)
    return (math.sin(d) * math.sin(a),
            math.sin(d) * math.cos(a),
            math.cos(d))


def jitter(v, s):
    return v + random.uniform(-s, s)


def make_patch(n, dip, dip_dir, cx, cy, cz, spread, normal_noise):
    """A roughly planar cluster of points sharing one joint orientation."""
    nx, ny, nz = plane_normal(dip, dip_dir)
    rows = []
    for _ in range(n):
        x = jitter(cx, spread)
        y = jitter(cy, spread)
        z = jitter(cz, spread)
        # perturb the normal slightly, then renormalise
        ax, ay, az = (jitter(nx, normal_noise),
                      jitter(ny, normal_noise),
                      jitter(nz, normal_noise))
        m = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        rows.append((x, y, z, ax / m, ay / m, az / m))
    return rows


def main():
    rows = []
    # Main slope face (background) — dip 70 / dir 140
    rows += make_patch(3000, SLOPE_DIP, SLOPE_DIR, 0, 0, 0, 9.0, 0.05)
    # J1 sliding set: shallow joints dipping the same way as the slope (27/140)
    rows += make_patch(900, 27, 140, 0, 4, 5, 6.0, 0.04)
    # J2 toppling set: steep joints dipping into the slope (85/320)
    rows += make_patch(700, 85, 320, -2, 2, 6, 5.0, 0.04)
    # J3 wedge partner: steep oblique set (80/250) — intersects J2 to form wedges
    rows += make_patch(500, 80, 250, 2, 3, 6, 5.0, 0.04)

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
