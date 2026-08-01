"""
camera_pose_sampler_v2.py -- fixes v1's azimuth bug: v1 sampled azimuth
relative to arbitrary world axes, so half the samples pointed away from
the robot entirely (into the background plane). This version builds a
local frame anchored on the real cluster->robot direction, so azimuth=0
always points toward the robot, and the sampled cone stays on the
robot's side by construction.
"""
import random
import math
import omni.usd as usd
from pxr import UsdGeom, Gf, Usd

stage = usd.get_context().get_stage()

CLUSTER_PATH = "/World/Cluster/Tomato_Cluster/Tomato_Cluster_Assembly"
LINK6_PATH = "/World/cr3/Geometry/world/dummy_link/base_link/Link1/Link2/Link3/Link4/Link5/Link6"
ROBOT_BASE_PATH = "/World/cr3/Geometry/world/dummy_link/base_link"
CAMERA_LOCAL_OFFSET = Gf.Vec3d(0.0, 0.0, 0.05)

DIST_RANGE = (0.15, 0.45)
AZIMUTH_RANGE_DEG = (-60, 60)     # now relative to the cluster->robot direction, not world X
ELEVATION_RANGE_DEG = (-10, 20)   # tightened - v1's 40deg max clipped into the trellis
WORLD_UP = Gf.Vec3d(0, 0, 1)

N_SAMPLES = 10
PLACE_DEBUG_SPHERES = True
DEBUG_ROOT = "/World/_PoseSamplerDebug"

cluster_prim = stage.GetPrimAtPath(CLUSTER_PATH)
robot_base_prim = stage.GetPrimAtPath(ROBOT_BASE_PATH)
if not cluster_prim.IsValid():
    raise RuntimeError(f"No prim at {CLUSTER_PATH}")
if not robot_base_prim.IsValid():
    raise RuntimeError(f"No prim at {ROBOT_BASE_PATH}")

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
box = bbox_cache.ComputeWorldBound(cluster_prim).ComputeAlignedBox()
cluster_center = (box.GetMin() + box.GetMax()) / 2.0

robot_base_pos = UsdGeom.Xformable(robot_base_prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()
).ExtractTranslation()

print(f"Cluster center: {cluster_center}")
print(f"Robot base position: {robot_base_pos}")

# Build a local frame anchored on the real cluster->robot direction,
# projected flat (horizontal) so "forward" doesn't tilt oddly based on
# the base's height vs the cluster's height.
to_robot = Gf.Vec3d(robot_base_pos[0] - cluster_center[0],
                     robot_base_pos[1] - cluster_center[1], 0.0)
if to_robot.GetLength() < 1e-6:
    raise RuntimeError("Robot base is directly above/below cluster center - "
                        "horizontal forward direction is undefined, fix manually")
forward = to_robot.GetNormalized()
right = Gf.Cross(forward, WORLD_UP).GetNormalized()
up = Gf.Cross(right, forward).GetNormalized()
print(f"Local frame - forward (toward robot): {forward}")


def sample_camera_pose():
    dist = random.uniform(*DIST_RANGE)
    az = math.radians(random.uniform(*AZIMUTH_RANGE_DEG))
    el = math.radians(random.uniform(*ELEVATION_RANGE_DEG))

    # az=0 now means "straight toward the robot base direction", not world +X
    offset = (dist * math.cos(el) * math.cos(az)) * forward \
            + (dist * math.cos(el) * math.sin(az)) * right \
            + (dist * math.sin(el)) * up
    cam_pos = cluster_center + offset

    view_matrix = Gf.Matrix4d().SetLookAt(cam_pos, cluster_center, WORLD_UP)
    cam_to_world = view_matrix.GetInverse()
    return cam_pos, cam_to_world


def link6_target_from_camera_pose(cam_to_world):
    cam_rot = cam_to_world.ExtractRotationMatrix()
    cam_pos = cam_to_world.ExtractTranslation()
    world_offset = cam_rot * CAMERA_LOCAL_OFFSET
    link6_pos = cam_pos - world_offset
    link6_rot = cam_rot
    return link6_pos, link6_rot


print(f"\n{'#':<4}{'cam_pos':<40}{'dist':<8}{'link6_pos'}")
print("-" * 95)

results = []
for i in range(N_SAMPLES):
    cam_pos, cam_to_world = sample_camera_pose()
    link6_pos, link6_rot = link6_target_from_camera_pose(cam_to_world)
    reconstructed = link6_pos + link6_rot * CAMERA_LOCAL_OFFSET
    error = (reconstructed - cam_pos).GetLength()
    results.append({"cam_pos": cam_pos, "link6_pos": link6_pos,
                     "link6_rot": link6_rot, "offset_check_error": error})
    dist_to_cluster = (cam_pos - cluster_center).GetLength()
    print(f"{i:<4}{str(tuple(round(v,3) for v in cam_pos)):<40}"
          f"{dist_to_cluster:<8.3f}{tuple(round(v,3) for v in link6_pos)}")

max_error = max(r["offset_check_error"] for r in results)
print(f"\nMax offset-math sanity check error: {max_error:.8f}")

if PLACE_DEBUG_SPHERES:
    if stage.GetPrimAtPath(DEBUG_ROOT):
        stage.RemovePrim(DEBUG_ROOT)
    for i, r in enumerate(results):
        path = f"{DEBUG_ROOT}/cam_{i}"
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.GetRadiusAttr().Set(0.01)
        UsdGeom.Xformable(sphere).AddTranslateOp().Set(r["cam_pos"])
        sphere.CreateDisplayColorAttr([(1.0, 1.0, 0.0)])
    cluster_marker = UsdGeom.Sphere.Define(stage, f"{DEBUG_ROOT}/cluster_center")
    cluster_marker.GetRadiusAttr().Set(0.015)
    UsdGeom.Xformable(cluster_marker).AddTranslateOp().Set(cluster_center)
    cluster_marker.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])
    robot_marker = UsdGeom.Sphere.Define(stage, f"{DEBUG_ROOT}/robot_base")
    robot_marker.GetRadiusAttr().Set(0.015)
    UsdGeom.Xformable(robot_marker).AddTranslateOp().Set(robot_base_pos)
    robot_marker.CreateDisplayColorAttr([(0.0, 0.0, 1.0)])  # blue = robot base, new reference point
    print(f"\nDebug spheres at {DEBUG_ROOT} - yellow=candidate cams, red=cluster center, blue=robot base")


