"""
FAA Core Algorithms
Directly ported from GEO Technical Note TN 4/2024, Appendix A
Extended Facet Amalgamation Approach for Rock Slope Kinematic Analysis
"""

import numpy as np
from scipy.spatial import KDTree


def plunge_and_trend_to_vector(p):
    """
    p: array of plunge and trend [n x 2], both in degree
    return the vector representing the plunge and trend
    note: the trend of the vertical vector may not be preserved
    """
    if len(p.shape) == 1:
        p = p[np.newaxis]

    p = np.radians(p)
    z = -np.sin(p[:, 0])
    xy = np.cos(p[:, 0])
    x = np.sin(p[:, 1]) * xy
    y = np.cos(p[:, 1]) * xy

    return np.vstack((x, y, z)).T


def vector_to_plunge_and_trend(n):
    """
    n: vectors [n x 3] (assumed to be normalised)
    return the plunge and trend [n x 2] represented by the vector
    in case of vertical vector i.e. [0 0 -1], the trend will be 0
    """
    if len(n.shape) == 1:
        n = n[np.newaxis]

    plunge = -np.arcsin(np.clip(n[:, 2], -1, 1))
    trend = np.arctan2(n[:, 0], n[:, 1])
    trend = np.mod(trend + 2 * np.pi, 2 * np.pi)
    return np.degrees(np.vstack((plunge, trend)).T)


def opposite_orientation_check(v, n):
    """
    v: the vector [1 x 3] or [n x 3] (assumed to be normalised) to be compared to
    n: the vectors [n x 3] (assumed to be normalised) being compared with vector v
    return a numpy boolean array of whether n is in opposite orientation to or perpendicular with v
    """
    return np.sum(n * v, axis=-1) <= 0


def dip_dip_direction_to_pole(d):
    """
    d: array of dip and dip direction (in degree)
    assume dip ranges from 0 to 90 and dip direction from 0 to 360
    return the plunge and trend of the pole
    """
    if len(d.shape) == 1:
        d = d[np.newaxis]
    plunge = 90 - d[:, 0]
    trend = np.mod(d[:, 1] + 180, 360)
    return np.vstack((plunge, trend)).T


def angle_difference(angle1, angle2):
    """
    compute the absolute difference between angles (in degrees),
    considering the case across the 0
    """
    return 180 - np.abs(np.abs(angle1 - angle2) - 180)


def sliding_check(n_raw, slope_dip, slope_dip_direction, friction_angle, lateral_limit):
    """
    n_raw: the normal vectors representing facet/points
    slope_dip: dip angle of the slope (in degree)
    slope_dip_direction: dip direction of the slope (in degree)
    friction_angle (in degree): friction angle of the joint surface
    lateral_limit (in degree): maximum angular difference of the dip direction of the planes and slope
    return a numpy boolean array of whether n is potentially subjected to sliding failure
    """
    n = n_raw.copy()
    if len(n.shape) == 1:
        n = n[np.newaxis]

    if slope_dip <= friction_angle:
        return None

    n[n[:, -1] > 0] *= -1
    d = np.array((slope_dip, slope_dip_direction))
    slope_pole = dip_dip_direction_to_pole(d)
    slope_pole_vec = plunge_and_trend_to_vector(slope_pole)
    facet_pole_plunge_and_trend = vector_to_plunge_and_trend(n)

    # Check lateral limit
    result = angle_difference(facet_pole_plunge_and_trend[:, 1], slope_pole[:, 1]) <= lateral_limit

    if result.sum() == 0:
        return None

    # Check friction angle
    outside_friction_cone = facet_pole_plunge_and_trend[result, 0] <= (90 - friction_angle)
    result[result] = outside_friction_cone

    if result.sum() == 0:
        return None

    # Check Daylight
    dip_vec = plunge_and_trend_to_vector(
        dip_dip_direction_to_pole(vector_to_plunge_and_trend(n[result]))
    )
    daylight = opposite_orientation_check(slope_pole_vec, dip_vec)
    result[result] = daylight

    if result.sum() == 0:
        return None

    return result


def toppling_check(n_raw, slope_dip, slope_dip_direction, friction_angle, lateral_limit):
    """
    n_raw: the normal vectors representing facet/points
    slope_dip: dip angle of the slope (in degree)
    slope_dip_direction: dip direction of the slope (in degree)
    friction_angle (in degree): friction angle of the joint surface
    lateral_limit (in degree): maximum angular difference of the dip direction of the planes and slope
    return a numpy boolean array of whether n is potentially subjected to flexural toppling
    """
    n = n_raw.copy()
    if len(n.shape) == 1:
        n = n[np.newaxis]

    if slope_dip <= friction_angle:
        return None

    n[n[:, -1] > 0] *= -1
    d = np.array((slope_dip, slope_dip_direction))
    facet_pole_plunge_and_trend = vector_to_plunge_and_trend(n)

    # Check lateral limit
    result = angle_difference(facet_pole_plunge_and_trend[:, 1], d[1]) <= lateral_limit

    if result.sum() == 0:
        return None

    # Check unstable_zone
    slope_pole = dip_dip_direction_to_pole(d)
    unstable_zone_pole = slope_pole + np.array((friction_angle, 0))
    unstable_zone_pole_vec = plunge_and_trend_to_vector(unstable_zone_pole)
    daylight = opposite_orientation_check(unstable_zone_pole_vec, n[result])
    result[result] = daylight

    if result.sum() == 0:
        return None

    return result


def wedge_check_triangular_mesh(facet, facet_normal, vertice, slope_dip,
                                 slope_dip_direction, friction_angle, min_angle_diff):
    """
    facet: array of indices of vertices defining the triangles
    facet_normal: the normal vector of each facet
    vertice: coordinates of the vertices
    slope_dip: dip angle of the slope (in degree)
    slope_dip_direction: dip direction of the slope (in degree)
    friction_angle (in degree): friction angle of the joint surface
    min_angle_diff (in degree): minimum angular difference for two facets to be considered wedge
    return edges (index of two vertices, coordinates of vertices) potentially subjected to wedge failure
    """
    if len(facet.shape) == 1:
        return (None, None)

    if slope_dip <= friction_angle:
        return (None, None)

    facet_ind = np.ones_like(facet, dtype=int).T * np.arange(facet.shape[0])

    edges = np.sort(np.vstack([facet.ravel(), np.roll(facet, -1, axis=-1).ravel()]).T)
    edges = np.vstack([edges.T, facet_ind.T.ravel()]).T
    del facet_ind
    edges = edges[np.lexsort((edges[:, 1], edges[:, 0]))]

    # Check whether there are two faces connected to an edge
    edges_vert = edges[:, :-1]
    two_facets = (edges_vert == np.roll(edges_vert, -1, axis=0)).all(axis=-1)
    edges = np.hstack((edges[two_facets],
                       edges[np.roll(two_facets, 1), -1].reshape(-1, 1)))
    del edges_vert, two_facets

    # Compute plunge and trend of the edge
    vec = vertice[edges[:, 1]] - vertice[edges[:, 0]]
    norms = np.linalg.norm(vec, axis=-1)
    valid = norms > 0
    if valid.sum() == 0:
        return (None, None)
    edges = edges[valid]
    vec = vec[valid]
    norms = norms[valid]
    vec = vec / norms.reshape(-1, 1)
    vec[vec[:, -1] > 0] *= -1
    plunge_trend = vector_to_plunge_and_trend(vec)

    # Check friction angle
    outside_friction_cone = plunge_trend[:, 0] >= friction_angle
    edges = edges[outside_friction_cone]
    vec = vec[outside_friction_cone]
    del plunge_trend, outside_friction_cone

    if edges.size == 0:
        return (None, None)

    # Check daylight
    d = np.array((slope_dip, slope_dip_direction))
    slope_pole = dip_dip_direction_to_pole(d)
    slope_pole_vec = plunge_and_trend_to_vector(slope_pole)
    daylight = opposite_orientation_check(slope_pole_vec, vec)
    edges = edges[daylight]
    del daylight, vec

    if edges.size == 0:
        return (None, None)

    # Check diverging/converging
    facet_1_centroid = vertice[facet[edges[:, 2]]].mean(axis=1)
    facet_2_centroid = vertice[facet[edges[:, 3]]].mean(axis=1)
    v12 = facet_2_centroid - facet_1_centroid
    v12_norms = np.linalg.norm(v12, axis=-1)
    valid = v12_norms > 0
    edges = edges[valid]
    facet_1_centroid = facet_1_centroid[valid]
    facet_2_centroid = facet_2_centroid[valid]
    v12 = v12[valid]
    v12_norms = v12_norms[valid]
    v12 = v12 / v12_norms.reshape(-1, 1)
    diverging = opposite_orientation_check(v12, facet_normal[edges[:, 2]])
    edges = edges[~diverging]
    del facet_1_centroid, facet_2_centroid, v12, diverging

    if edges.size == 0:
        return (None, None)

    # Check angular difference
    cos_angle_between_facets = np.sum(facet_normal[edges[:, 2]] * facet_normal[edges[:, 3]], axis=1)
    cos_min_angle_diff = np.cos(np.radians(min_angle_diff))
    edges = edges[cos_angle_between_facets <= cos_min_angle_diff]
    del cos_angle_between_facets

    if edges.size == 0:
        return (None, None)

    unique_vert, inverse = np.unique(edges[:, :2], return_inverse=True)
    return (inverse.reshape(-1, 2), vertice[unique_vert])


def wedge_check_point(coors, n, slope_dip, slope_dip_direction,
                       friction_angle, min_angle_diff, k):
    """
    coors: array of points (x, y, z)
    n: the normal vector of each point
    slope_dip: dip angle of the slope (in degree)
    slope_dip_direction: dip direction of the slope (in degree)
    friction_angle (in degree): friction angle of the joint surface
    min_angle_diff (in degree): minimum angular difference for two planes to be considered wedge
    k: number of nearest neighbours
    return points near to the edges potentially subjected to wedge failure
    """
    if len(coors.shape) == 1:
        return None

    if coors.shape[0] < k + 1:
        return None

    if slope_dip <= friction_angle:
        return None

    tree = KDTree(coors)
    _, n_idx = tree.query(coors, k + 1)

    idx = np.repeat(n_idx[:, 0], k)
    pairs = np.vstack((idx, n_idx[:, 1:].ravel())).T
    # Keep only pairs whose indices are within the normal array bounds
    valid_idx = (pairs[:, 0] < len(n)) & (pairs[:, 1] < len(n))
    pairs = pairs[valid_idx]
    pairs.sort(axis=-1)
    pairs = np.unique(pairs, axis=0)
    del idx, n_idx

    # Plunge and trend of the line of intersections
    cross_product = np.cross(n[pairs[:, 0]], n[pairs[:, 1]])
    cross_product[cross_product[:, -1] > 0] *= -1
    norm = np.linalg.norm(cross_product, axis=-1)
    cross_product = cross_product[norm != 0]
    pairs = pairs[norm != 0]
    norm = norm[norm != 0]
    cross_product = cross_product / norm.reshape(-1, 1)
    plunge_trend = vector_to_plunge_and_trend(cross_product)

    # Check friction angle
    outside_friction_cone = plunge_trend[:, 0] >= friction_angle
    pairs = pairs[outside_friction_cone]
    cross_product = cross_product[outside_friction_cone]
    del plunge_trend, outside_friction_cone

    if pairs.size == 0:
        return None

    # Check daylight
    d = np.array((slope_dip, slope_dip_direction))
    slope_pole = dip_dip_direction_to_pole(d)
    slope_pole_vec = plunge_and_trend_to_vector(slope_pole)
    daylight = opposite_orientation_check(slope_pole_vec, cross_product)
    pairs = pairs[daylight]
    cross_product = cross_product[daylight]
    del daylight

    if pairs.size == 0:
        return None

    # Check diverging/converging
    d12 = coors[pairs[:, 1]] - coors[pairs[:, 0]]
    d12_norms = np.linalg.norm(d12, axis=-1)
    valid = d12_norms > 0
    pairs = pairs[valid]
    cross_product = cross_product[valid]
    d12 = d12[valid]
    d12_norms = d12_norms[valid]
    d12 = d12 / d12_norms.reshape(-1, 1)
    v12dotn1 = np.sum(n[pairs[:, 0]] * d12, axis=-1)
    diverging = v12dotn1 <= 0
    pairs = pairs[~diverging]
    cross_product = cross_product[~diverging]
    del diverging, v12dotn1, d12

    if pairs.size == 0:
        return None

    # Check angular difference
    cos_angle_between = np.sum(n[pairs[:, 0]] * n[pairs[:, 1]], axis=-1)
    cos_min_angle_diff = np.cos(np.radians(min_angle_diff))
    different_jointsets = cos_angle_between <= cos_min_angle_diff
    pairs = pairs[different_jointsets]
    cross_product = cross_product[different_jointsets]
    del cos_angle_between, different_jointsets

    if pairs.size == 0:
        return None

    # Mid-Point calculation
    A = np.concatenate((n[pairs[:, 0]], n[pairs[:, 1]], cross_product),
                       axis=-1).reshape(-1, 3, 3)
    try:
        invA = np.linalg.inv(A)
    except np.linalg.LinAlgError:
        return None

    p1_dot_cross_product = np.sum(coors[pairs[:, 0]] * cross_product, axis=-1)
    p2_dot_cross_product = np.sum(coors[pairs[:, 1]] * cross_product, axis=-1)

    constants = np.vstack((
        np.sum(coors[pairs[:, 0]] * n[pairs[:, 0]], axis=-1),
        np.sum(coors[pairs[:, 1]] * n[pairs[:, 1]], axis=-1),
        p1_dot_cross_product
    )).T
    p1p = np.matmul(invA, constants[..., np.newaxis]).reshape(-1, 3)
    constants[:, -1] = p2_dot_cross_product
    p2p = np.matmul(invA, constants[..., np.newaxis]).reshape(-1, 3)

    return (p1p + p2p) / 2


def fit_plane_to_points(points):
    """
    Fit a plane to a set of 3D points using PCA.
    Returns (dip, dip_direction) in degrees.
    """
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered)
    normal = vh[-1]
    if normal[2] > 0:
        normal = -normal
    pt = vector_to_plunge_and_trend(normal)
    plunge, trend = pt[0]
    dip = 90 - plunge
    dip_direction = (trend + 180) % 360
    return dip, dip_direction
