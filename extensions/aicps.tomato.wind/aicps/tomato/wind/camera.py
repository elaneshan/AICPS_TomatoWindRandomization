import random, math
from pxr import Usd, UsdGeom, Gf

CLUSTER_ROOT = "/World/Cluster/Tomato_Cluster/Tomato_Cluster_Assembly"
CAMERA_PATH = "/World/OverviewCam"
BACKGROUND_PATH = "/World/Cluster/Tomato_Cluster/background_plane_geo"

WORLD_UP = Gf.Vec3d(0, 0, 1)  # Z-up


def get_cluster_bounds(stage, root_path=CLUSTER_ROOT):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise RuntimeError(f"Cluster root not found at {root_path}")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = bbox_cache.ComputeWorldBound(root).ComputeAlignedBox()
    center = (box.GetMin() + box.GetMax()) / 2.0
    size = box.GetMax() - box.GetMin()
    radius = max(size[0], size[1], size[2]) / 2.0
    return center, radius


def compute_front_azimuth(stage, background_path=BACKGROUND_PATH, cluster_root=CLUSTER_ROOT):
    """Front direction = vector from the background plane toward the
    cluster, projected onto the XY plane (Z-up world). Returns the
    azimuth (degrees) that faces directly away from the backdrop -
    i.e. the direction the camera should be centered around so it's
    always looking at the fruit-facing side of the cluster."""
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    bg_prim = stage.GetPrimAtPath(background_path)
    if not bg_prim.IsValid():
        raise RuntimeError(f"Background plane not found at {background_path}")
    bg_box = bbox_cache.ComputeWorldBound(bg_prim).ComputeAlignedBox()
    bg_center = (bg_box.GetMin() + bg_box.GetMax()) / 2.0

    cluster_center, _ = get_cluster_bounds(stage, cluster_root)

    direction = cluster_center - bg_center
    return math.degrees(math.atan2(direction[1], direction[0]))


def set_look_at(xformable, position, target, up=WORLD_UP):
    forward = (target - position).GetNormalized()
    right = Gf.Cross(forward, up)
    if right.GetLength() < 1e-6:
        fallback_up = Gf.Vec3d(0, 1, 0) if up != Gf.Vec3d(0, 1, 0) else Gf.Vec3d(1, 0, 0)
        right = Gf.Cross(forward, fallback_up)
    right = right.GetNormalized()
    true_up = Gf.Cross(right, forward).GetNormalized()

    mat = Gf.Matrix4d(1.0)
    mat.SetRow(0, Gf.Vec4d(right[0], right[1], right[2], 0.0))
    mat.SetRow(1, Gf.Vec4d(true_up[0], true_up[1], true_up[2], 0.0))
    mat.SetRow(2, Gf.Vec4d(-forward[0], -forward[1], -forward[2], 0.0))
    mat.SetRow(3, Gf.Vec4d(position[0], position[1], position[2], 1.0))

    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(mat)


def point_in_frustum(frustum, point):
    """Rough secondary check - bbox-center-in-frustum only, no occlusion
    awareness. Kept as a loose sanity gate; the real fix for occlusion
    is the future segmentation-based filter, not this."""
    view = frustum.ComputeViewMatrix()
    proj = frustum.ComputeProjectionMatrix()
    combined = view * proj
    ndc = combined.Transform(point)
    return -1.0 <= ndc[0] <= 1.0 and -1.0 <= ndc[1] <= 1.0 and -1.0 <= ndc[2] <= 1.0


def create_overview_camera(stage, path=CAMERA_PATH):
    camera = UsdGeom.Camera.Define(stage, path)
    camera.GetFocalLengthAttr().Set(24.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
    return camera


def randomize_overview_camera(
    stage,
    front_azimuth_deg=None,       # if None, computed fresh each call from BACKGROUND_PATH
    azimuth_spread=90.0,          # +/- from front = 180 total sweep
    elevation_range=(-25.0, 65.0),
    distance_factor_range=(1.5, 4.0),
    look_at_jitter_frac=0.15,
):
    center, radius = get_cluster_bounds(stage)

    if front_azimuth_deg is None:
        front_azimuth_deg = compute_front_azimuth(stage)

    camera_prim = stage.GetPrimAtPath(CAMERA_PATH)
    if not camera_prim.IsValid():
        camera_prim = create_overview_camera(stage).GetPrim()

    azimuth_range = (front_azimuth_deg - azimuth_spread, front_azimuth_deg + azimuth_spread)
    azimuth = math.radians(random.uniform(*azimuth_range))
    elevation = math.radians(random.uniform(*elevation_range))
    distance = radius * random.uniform(*distance_factor_range)

    position = Gf.Vec3d(
        center[0] + distance * math.cos(elevation) * math.cos(azimuth),
        center[1] + distance * math.cos(elevation) * math.sin(azimuth),
        center[2] + distance * math.sin(elevation),
    )

    jitter = radius * look_at_jitter_frac
    target = Gf.Vec3d(
        center[0] + random.uniform(-jitter, jitter),
        center[1] + random.uniform(-jitter, jitter),
        center[2] + random.uniform(-jitter, jitter),
    )

    set_look_at(UsdGeom.Xformable(camera_prim), position, target)

    return {
        "position": [position[0], position[1], position[2]],
        "target": [target[0], target[1], target[2]],
        "azimuth_deg": math.degrees(azimuth),
        "elevation_deg": math.degrees(elevation),
        "distance": distance,
        "front_azimuth_deg": front_azimuth_deg,
    }





def frame_has_visible_fruit(stage, camera_path=CAMERA_PATH, cluster_root=CLUSTER_ROOT):
    camera_prim = stage.GetPrimAtPath(camera_path)
    camera = UsdGeom.Camera(camera_prim)
    frustum = camera.GetCamera(Usd.TimeCode.Default()).frustum

    root = stage.GetPrimAtPath(cluster_root)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    for prim in Usd.PrimRange(root):
        if "tomato" not in prim.GetName().lower():
            continue
        box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        center = (box.GetMin() + box.GetMax()) / 2.0
        if point_in_frustum(frustum, center):
            return True
    return False


def randomize_overview_camera_with_fruit(stage, max_tries=15, **kwargs):
    info = None
    for attempt in range(max_tries):
        info = randomize_overview_camera(stage, **kwargs)
        if frame_has_visible_fruit(stage):
            info["fruit_visible"] = True
            info["attempts"] = attempt + 1
            return info

    print(f"WARNING: no fruit-visible camera pose found after {max_tries} tries")
    info["fruit_visible"] = False
    info["attempts"] = max_tries
    return info





