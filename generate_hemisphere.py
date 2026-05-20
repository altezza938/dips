"""
Generate the canonical hemisphere validation model as a plain-text .xyzn file
(x y z nx ny nz per line). No third-party libraries required.

A hemisphere (dome) contains every possible surface orientation, so running
the FAA analysis on it paints clean contiguous zones — sliding on the
slope-facing side, toppling on the opposite side — exactly like the GEO
TN 4/2024 reference tool's hemisphere example. Use it to verify the app.

Run:  python generate_hemisphere.py
"""

import math, random

random.seed(1)
N = 12000          # surface points
R = 10.0           # radius

rows = []
while len(rows) < N:
    # uniform direction on the sphere, keep upper hemisphere (dome)
    x = random.gauss(0, 1); y = random.gauss(0, 1); z = random.gauss(0, 1)
    m = math.sqrt(x*x + y*y + z*z)
    if m == 0:
        continue
    x, y, z = x/m, y/m, z/m
    if z < 0:
        z = -z          # fold to upper hemisphere
    # point on the sphere surface; outward normal = radial direction
    rows.append((R*x, R*y, R*z, x, y, z))

with open("hemisphere.xyzn", "w") as f:
    f.write("# x y z nx ny nz  — hemisphere validation model (radial normals)\n")
    for x, y, z, nx, ny, nz in rows:
        f.write(f"{x:.4f} {y:.4f} {z:.4f} {nx:.4f} {ny:.4f} {nz:.4f}\n")

print(f"Saved hemisphere.xyzn ({len(rows):,} points)")
print("Suggested test: turn OFF 'Amalgamate into facets' (every point is a")
print("unique orientation), set Friction 30, then click Run All. You should")
print("see a sliding (red) band and a toppling (green) band on opposite sides.")
