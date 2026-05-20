"""
Generate a synthetic rock slope point cloud for testing the FAA application.
Produces a curved slope face with simulated joint sets.
Run:  python generate_sample.py
"""

import numpy as np
import open3d as o3d

np.random.seed(42)


def make_slope_surface(n=4000, dip=70, dip_dir=140):
    """Curved slope face — main background surface."""
    dip_rad = np.radians(dip)
    dir_rad = np.radians(dip_dir)

    u = np.random.uniform(-10, 10, n)
    v = np.random.uniform(0, 20, n)

    x = u * np.cos(dir_rad) + v * np.sin(dir_rad) * np.cos(dip_rad)
    y = -u * np.sin(dir_rad) + v * np.cos(dir_rad) * np.cos(dip_rad)
    z = v * np.sin(dip_rad) + np.random.normal(0, 0.05, n)

    normals = np.tile(
        [-np.sin(dip_rad) * np.sin(dir_rad),
         -np.sin(dip_rad) * np.cos(dir_rad),
          np.cos(dip_rad)], (n, 1))
    normals += np.random.normal(0, 0.05, normals.shape)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)

    return np.column_stack([x, y, z]), normals


def make_joint_set(n, dip, dip_dir, x_range, y_range, z_range, noise=0.08):
    """Flat joint plane within a bounding box."""
    dip_rad = np.radians(dip)
    dir_rad = np.radians(dip_dir)

    u = np.random.uniform(*x_range, n)
    v = np.random.uniform(*y_range, n)

    x = u
    y = v
    z_base = (np.random.uniform(*z_range, n) +
               np.tan(dip_rad) * (u * np.sin(dir_rad) + v * np.cos(dir_rad)))
    z = z_base + np.random.normal(0, noise, n)

    normals = np.tile(
        [-np.sin(dip_rad) * np.sin(dir_rad),
         -np.sin(dip_rad) * np.cos(dir_rad),
          np.cos(dip_rad)], (n, 1))
    normals += np.random.normal(0, noise, normals.shape)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)

    return np.column_stack([x, y, z]), normals


def main():
    # --- Background slope (dip 70° / dip direction 140°) ---
    pts_s, nrm_s = make_slope_surface(4000, dip=70, dip_dir=140)

    # --- J1: Subhorizontal sliding joints (dip 27° / dip dir 144°) ---
    pts_j1, nrm_j1 = make_joint_set(800, 27, 144, (-8, 8), (2, 18), (0, 15))

    # --- J2: Steep toppling joints (dip 87° / dip dir 008°) ---
    pts_j2, nrm_j2 = make_joint_set(600, 87, 8, (-6, 6), (4, 16), (0, 15))

    # --- J3: Wedge partner joints (dip 81° / dip dir 254°) ---
    pts_j3, nrm_j3 = make_joint_set(400, 81, 254, (-5, 5), (6, 14), (0, 15))

    all_pts = np.vstack([pts_s, pts_j1, pts_j2, pts_j3])
    all_nrm = np.vstack([nrm_s, nrm_j1, nrm_j2, nrm_j3])

    # Colour by source
    colours = np.vstack([
        np.tile([0.7, 0.7, 0.7], (len(pts_s),  1)),
        np.tile([1.0, 0.5, 0.2], (len(pts_j1), 1)),
        np.tile([0.2, 0.8, 0.2], (len(pts_j2), 1)),
        np.tile([0.2, 0.6, 1.0], (len(pts_j3), 1)),
    ])

    pcd = o3d.geometry.PointCloud()
    pcd.points  = o3d.utility.Vector3dVector(all_pts)
    pcd.normals = o3d.utility.Vector3dVector(all_nrm)
    pcd.colors  = o3d.utility.Vector3dVector(colours)

    out = 'sample_slope.ply'
    o3d.io.write_point_cloud(out, pcd)
    print(f'Saved {out}  ({len(all_pts):,} points)')
    print()
    print('Suggested parameters for this sample:')
    print('  Slope Dip:          70°')
    print('  Slope Dip Direction: 140°')
    print('  Friction Angle:      30°')
    print('  Lateral Limit (Sliding): ±20°')
    print('  Lateral Limit (Toppling): ±10°')
    print('  Min. Angular Diff (Wedge): 30°')


if __name__ == '__main__':
    main()
