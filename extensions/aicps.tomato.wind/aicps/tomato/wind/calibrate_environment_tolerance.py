from pxr import UsdGeom
import aicps.tomato.wind.mesh_distance as mesh_distance


def clear_prim_pair_markers(stage, debug_root="/World/_PrimPairMarkers"):
    if stage.GetPrimAtPath(debug_root):
        stage.RemovePrim(debug_root)


def place_prim_pair_markers(prim_a, prim_b, stage, marker_radius=0.002,
                             debug_root="/World/_PrimPairMarkers"):
    """
    Generalized closest-point marker tool - works for any two prims, not
    just pedicel-vs-pedicel. Use this to calibrate the trellis tolerance
    the same way fruit/stem tolerances were calibrated: find a pedicel
    part that's close to the trellis, check the reported distance, look
    at the markers, and decide if that distance is real contact or a
    real gap.

    Example:
        tomato = collisions.find_child_by_prefix(pedicel, "tomato")
        place_prim_pair_markers(tomato, trellis_prim, stage)
    """
    clear_prim_pair_markers(stage, debug_root)

    info = mesh_distance.min_distance_verbose(prim_a, prim_b)
    if info is None:
        print("WARNING: min_distance_verbose returned no results")
        return None

    print(f"reported distance: {info['distance']:.5f}")
    print(f"  point A ({info['mesh_a_path']}): {info['point_a']}")
    print(f"  point B ({info['mesh_b_path']}): {info['point_b']}")

    for name, point, color in [
        ("A", info["point_a"], (1.0, 0.0, 1.0)),   # magenta
        ("B", info["point_b"], (0.0, 1.0, 1.0)),   # cyan
    ]:
        path = f"{debug_root}/{name}"
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.GetRadiusAttr().Set(marker_radius)
        UsdGeom.Xformable(sphere).AddTranslateOp().Set(tuple(point))
        sphere.CreateDisplayColorAttr([color])
        sphere.CreateDisplayOpacityAttr([1.0])

    print(f"\nMarkers placed at {debug_root}/A (magenta) and {debug_root}/B (cyan).")
    return info


