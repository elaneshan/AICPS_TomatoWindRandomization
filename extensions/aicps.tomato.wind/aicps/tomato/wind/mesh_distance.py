import numpy as np
from pxr import Usd, UsdGeom
import omni.usd

def get_world_points_with_provenance(prim):
    """
    Same traversal as get_world_points, but tags every point with the
    prim path it came from. Slower (Python-level loop instead of pure
    numpy concat) - diagnostic use only, not for the hot randomization
    loop.
    """
    entries = []
    for descendant in Usd.PrimRange(prim):
        if not descendant.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(descendant)
        local_points = mesh.GetPointsAttr().Get()
        if not local_points:
            continue
        world_matrix = UsdGeom.Xformable(descendant).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        path_str = str(descendant.GetPath())
        for p in local_points:
            wp = world_matrix.Transform(p)
            entries.append((np.array([wp[0], wp[1], wp[2]], dtype=np.float64), path_str))
    return entries


def get_combined_world_points(prims):
    """
    Concatenates world-space points from multiple separate prims into one
    array - for treating a group of independent prims (e.g. all 14 leaves)
    as one static collision target, same idea as get_world_points does
    for meshes within a single prim's subtree.
    """
    all_points = []
    for prim in prims:
        pts = get_world_points(prim)
        if pts is not None:
            all_points.append(pts)
    if not all_points:
        return None
    return np.concatenate(all_points, axis=0)


def min_distance_verbose(prim_a, prim_b, sample_stride=1):
    """
    Returns the true minimum distance PLUS which mesh prim each of the
    two closest points came from and their world positions. Use this to
    confirm whether a flagged pair's minimum is coming from actual fruit
    surface geometry or from something unexpected nested in the subtree
    (calyx, stem cap, etc.).
    """
    entries_a = get_world_points_with_provenance(prim_a)
    entries_b = get_world_points_with_provenance(prim_b)
    if not entries_a or not entries_b:
        return None

    pts_a = np.array([e[0] for e in entries_a])[::sample_stride]
    paths_a = [e[1] for e in entries_a][::sample_stride]
    pts_b = np.array([e[0] for e in entries_b])[::sample_stride]
    paths_b = [e[1] for e in entries_b][::sample_stride]

    diffs = pts_a[:, None, :] - pts_b[None, :, :]
    dists = np.sqrt(np.sum(diffs ** 2, axis=-1))
    i, j = np.unravel_index(np.argmin(dists), dists.shape)

    return {
        "distance": float(dists[i, j]),
        "point_a": pts_a[i].tolist(),
        "point_b": pts_b[j].tolist(),
        "mesh_a_path": paths_a[i],
        "mesh_b_path": paths_b[j],
    }




def get_world_points(prim):
    """
    Collects world-space vertex positions for a prim's subtree. Traverses
    down to find every UsdGeom.Mesh (the tomato prim itself might be an
    Xform wrapping the actual mesh, not a mesh directly), transforms each
    mesh's local points into world space, and concatenates them all.
    """
    points_list = []
    for descendant in Usd.PrimRange(prim):
        if not descendant.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(descendant)
        local_points = mesh.GetPointsAttr().Get()
        if not local_points:
            continue
        world_matrix = UsdGeom.Xformable(descendant).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pts = np.array([world_matrix.Transform(p) for p in local_points], dtype=np.float64)
        points_list.append(pts)

    if not points_list:
        return None
    return np.concatenate(points_list, axis=0)


def min_distance(prim_a, prim_b, sample_stride=1):
    """
    True minimum point-to-point distance between two meshes' vertices.
    sample_stride > 1 subsamples points for speed on high-poly meshes
    (e.g. stride=5 uses every 5th vertex) - use 1 for the ground-truth
    calibration pass, increase later if this needs to run per-candidate
    during randomization and full precision is too slow.
    """
    pts_a = get_world_points(prim_a)
    pts_b = get_world_points(prim_b)
    if pts_a is None or pts_b is None:
        return None

    pts_a = pts_a[::sample_stride]
    pts_b = pts_b[::sample_stride]

    # Brute-force pairwise distance (no scipy dependency assumed).
    # For a few hundred/thousand verts per tomato this is still fast enough
    # for a one-off diagnostic; revisit if it's too slow at full mesh res.
    diffs = pts_a[:, None, :] - pts_b[None, :, :]
    dists = np.sqrt(np.sum(diffs ** 2, axis=-1))
    return float(dists.min())

def min_distance_from_cached_points(cached_points_a, prim_b, sample_stride=1):
    """
    Same as min_distance, but the first mesh's world points are already
    computed (passed in directly) instead of re-traversing its prim
    subtree. Use this for checks against a STATIC object (like a
    trellis) whose points never change between calls - recomputing them
    every time would be wasted work, especially for a deeply nested
    hierarchy.
    """
    if cached_points_a is None:
        return None
    pts_b = get_world_points(prim_b)
    if pts_b is None:
        return None

    pts_b = pts_b[::sample_stride]
    diffs = cached_points_a[:, None, :] - pts_b[None, :, :]
    dists = np.sqrt(np.sum(diffs ** 2, axis=-1))
    return float(dists.min())
