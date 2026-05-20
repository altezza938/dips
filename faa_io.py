"""
FAA I/O Module
Loads point clouds and triangle meshes using open3d.
Supports: PLY, OBJ, STL, FBX, OFF, GLTF, GLB (mesh)
          XYZ, XYZN, XYZRGB, PLY, PCD, PTS (point cloud)
          LAS, LAZ (LiDAR point cloud — requires laspy[lazrs])
"""

import numpy as np
import open3d as o3d


MESH_EXTENSIONS  = {'.ply', '.obj', '.stl', '.fbx', '.off', '.gltf', '.glb'}
CLOUD_EXTENSIONS = {'.xyz', '.xyzn', '.xyzrgb', '.pcd', '.pts'}
LAS_EXTENSIONS   = {'.las', '.laz'}

NEIGHBOUR_SIZE_FOR_NORMALS = 0.2  # metres, default from TN 4/2024

# Candidate normal attribute names written by GeoSLAM Hub and other common tools
_NORMAL_DIM_CANDIDATES = [
    ('NormalX', 'NormalY', 'NormalZ'),
    ('normal_x', 'normal_y', 'normal_z'),
    ('nx', 'ny', 'nz'),
]


def load_file(filepath):
    """
    Load a 3D file and return (data_type, geometry).
    data_type: 'mesh' or 'point_cloud'
    geometry: dict with keys:
        For mesh:    'vertices', 'triangles', 'normals' (per-triangle), 'vertex_normals'
        For cloud:   'points', 'normals' (per-point), 'colors'
    """
    import os
    ext = os.path.splitext(filepath)[1].lower()

    if ext in LAS_EXTENSIONS:
        return _load_las(filepath)
    elif ext in MESH_EXTENSIONS:
        return _load_mesh(filepath)
    else:
        return _load_point_cloud(filepath)


def _load_mesh(filepath):
    mesh = o3d.io.read_triangle_mesh(filepath)
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()

    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    tri_normals = np.asarray(mesh.triangle_normals)
    vert_normals = np.asarray(mesh.vertex_normals)

    # Colours (grey if none)
    if mesh.has_vertex_colors():
        colors = np.asarray(mesh.vertex_colors)
    else:
        colors = np.ones((len(vertices), 3)) * 0.7

    return 'mesh', {
        'vertices': vertices,
        'triangles': triangles,
        'normals': tri_normals,
        'vertex_normals': vert_normals,
        'colors': colors,
    }


def _load_point_cloud(filepath):
    pcd = o3d.io.read_point_cloud(filepath)

    points = np.asarray(pcd.points)

    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamRadius(
                radius=NEIGHBOUR_SIZE_FOR_NORMALS
            )
        )
        pcd.orient_normals_consistent_tangent_plane(k=15)

    normals = np.asarray(pcd.normals)

    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
    else:
        colors = np.ones((len(points), 3)) * 0.7

    return 'point_cloud', {
        'points': points,
        'normals': normals,
        'colors': colors,
    }


def _load_las(filepath):
    """Load a LAS/LAZ file using laspy."""
    try:
        import laspy
    except ImportError:
        raise ImportError(
            "laspy is required for LAS/LAZ files. "
            "Install it with:  pip install 'laspy[lazrs]>=2.0'"
        )

    las = laspy.read(filepath)

    points = np.vstack([
        np.asarray(las.x, dtype=np.float64),
        np.asarray(las.y, dtype=np.float64),
        np.asarray(las.z, dtype=np.float64),
    ]).T

    # Colours — LAS stores RGB as 16-bit; normalise to [0, 1]
    dim_names = set(las.point_format.dimension_names)
    if {'red', 'green', 'blue'}.issubset(dim_names):
        scale = 65535.0
        colors = np.vstack([
            np.asarray(las.red,   dtype=np.float64) / scale,
            np.asarray(las.green, dtype=np.float64) / scale,
            np.asarray(las.blue,  dtype=np.float64) / scale,
        ]).T
        colors = np.clip(colors, 0.0, 1.0)
    else:
        # Fall back to intensity-mapped grey if available
        if 'intensity' in dim_names:
            intensity = np.asarray(las.intensity, dtype=np.float64)
            mx = intensity.max()
            if mx > 0:
                grey = intensity / mx
            else:
                grey = np.ones(len(points)) * 0.7
            colors = np.column_stack([grey, grey, grey])
        else:
            colors = np.ones((len(points), 3)) * 0.7

    # Normals — check for extra dimensions written by GeoSLAM Hub / other tools
    normals = None
    extra_dims = set(las.point_format.extra_dimension_names)
    for nx_name, ny_name, nz_name in _NORMAL_DIM_CANDIDATES:
        if {nx_name, ny_name, nz_name}.issubset(extra_dims):
            normals = np.vstack([
                np.asarray(las[nx_name], dtype=np.float64),
                np.asarray(las[ny_name], dtype=np.float64),
                np.asarray(las[nz_name], dtype=np.float64),
            ]).T
            break

    if normals is None:
        normals = _estimate_normals_open3d(points)

    return 'point_cloud', {
        'points':  points,
        'normals': normals,
        'colors':  colors,
    }


def _estimate_normals_open3d(points, radius=NEIGHBOUR_SIZE_FOR_NORMALS):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamRadius(radius=radius)
    )
    pcd.orient_normals_consistent_tangent_plane(k=15)
    return np.asarray(pcd.normals)


def estimate_normals_for_cloud(points, radius=NEIGHBOUR_SIZE_FOR_NORMALS):
    """Estimate per-point normals from a numpy array of XYZ points."""
    return _estimate_normals_open3d(points, radius)


def save_point_cloud(filepath, points, colors=None):
    """Save a point cloud to file."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(filepath, pcd)


def save_mesh(filepath, vertices, triangles, colors=None):
    """Save a triangle mesh to file."""
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    if colors is not None:
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(filepath, mesh)


def get_bounding_box(data_type, geometry):
    """Return (min_xyz, max_xyz) of the geometry."""
    if data_type == 'mesh':
        pts = geometry['vertices']
    else:
        pts = geometry['points']
    return pts.min(axis=0), pts.max(axis=0)


def get_centroid(data_type, geometry):
    if data_type == 'mesh':
        return geometry['vertices'].mean(axis=0)
    else:
        return geometry['points'].mean(axis=0)
