"""
camera_pose_sampler_v2.py -- fixes v1's azimuth bug: v1 sampled azimuth
relative to arbitrary world axes, so half the samples pointed away from
the robot entirely (into the background plane). This version builds a
local frame anchored on the real cluster->robot direction, so azimuth=0
always points toward the robot, and the sampled cone stays on the
robot's side by construction.


SEED is now read from the SAMPLER_SEED env var (defaults to 42, so
running this standalone in the Kit Script Editor behaves exactly as
before). export_sampled_poses.py sets this env var before loading the
module, once per batch, to generate multiple independent samples.
"""
import os
import random
import math
import omni.usd as usd
from pxr import UsdGeom as UsdGeom
from pxr import Gf as Gf
from pxr import Usd as Usd




SEED = int(os.environ.get("SAMPLER_SEED", "42"))
random.seed(SEED)




stage = usd.get_context().get_stage()




CLUSTER_PATH = "/World/Cluster/Tomato_Cluster/Tomato_Cluster_Assembly"
LINK6_PATH = "/World/cr3/Geometry/world/dummy_link/base_link/Link1/Link2/Link3/Link4/Link5/Link6"
ROBOT_BASE_PATH = "/World/cr3/Geometry/world/dummy_link/base_link"


# NOTE: this constant is now vestigial -- CAMERA_LOCAL_TO_LINK6 below is
# re-derived from the live stage every run, which is what actually gets
# used. Left in only so nothing else in the file breaks if it's referenced
# elsewhere; not a source of truth anymore (hasn't been since the v11 fix).
CAMERA_LOCAL_OFFSET = Gf.Vec3d(0.0, 0.0, 0.05)




DIST_RANGE = (0.15, 0.45)
AZIMUTH_RANGE_DEG = (-60, 60)     # relative to the cluster->robot direction, not world X
AZIMUTH_DEADZONE_DEG = (-20, 20)  # avoid sampling directly in front of the robot, where it can't see the cluster
ELEVATION_RANGE_DEG = (-10, 20)   # tightened - v1's 40deg max clipped into the trellis
WORLD_UP = Gf.Vec3d(0, 0, 1)




N_SAMPLES = 10
PLACE_DEBUG_SPHERES = False  # set True to visualize sampled camera positions in the stage
DEBUG_ROOT = "/World/_PoseSamplerDebug"




cluster_prim = stage.GetPrimAtPath(CLUSTER_PATH)
robot_base_prim = stage.GetPrimAtPath(ROBOT_BASE_PATH)
if not cluster_prim.IsValid():
    raise RuntimeError(f"No prim at {CLUSTER_PATH}")
if not robot_base_prim.IsValid():
    raise RuntimeError(f"No prim at {ROBOT_BASE_PATH}")


# UPDATED: camera is now EyeInHand_Camera (mounted on/near the gripper
# housing), not WristCamera (the old Link6-only mount from v8/v9).
CAMERA_PATH = "/World/cr3/Geometry/world/dummy_link/base_link/Link1/Link2/Link3/Link4/Link5/Link6/Gripper/Geometry/gripper_base_link/EyeInHand_Camera"
camera_prim = stage.GetPrimAtPath(CAMERA_PATH)
camera = UsdGeom.Camera(camera_prim)
camera.GetFocalLengthAttr().Set(12.5)  # override the default 50mm to match the real camera
link6_prim = stage.GetPrimAtPath(LINK6_PATH)
if not camera_prim.IsValid():
    raise RuntimeError(f"No prim at {CAMERA_PATH}")
if not link6_prim.IsValid():
    raise RuntimeError(f"No prim at {LINK6_PATH}")


# Real, current local transform of the camera relative to Link6, read
# from the stage rather than assumed. Re-deriving it fresh here means
# today's new mount offset/rotation is picked up automatically.
cam_to_world_at_load = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
link6_to_world_at_load = UsdGeom.Xformable(link6_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
CAMERA_LOCAL_TO_LINK6 = cam_to_world_at_load * link6_to_world_at_load.GetInverse()


print(f"[seed={SEED}] Camera local-to-Link6 (re-derived from stage this run):")
print(f"  translate: {CAMERA_LOCAL_TO_LINK6.ExtractTranslation()}")
print(f"  rotation deviation from identity: "
      f"{sum((CAMERA_LOCAL_TO_LINK6.ExtractRotationMatrix() - Gf.Matrix3d(1.0)).GetRow(r).GetLength() for r in range(3)):.4f}")




bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
box = bbox_cache.ComputeWorldBound(cluster_prim).ComputeAlignedBox()
cluster_center = (box.GetMin() + box.GetMax()) / 2.0




robot_base_pos = UsdGeom.Xformable(robot_base_prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()
).ExtractTranslation()




print(f"[seed={SEED}] Cluster center: {cluster_center}")
print(f"[seed={SEED}] Robot base position: {robot_base_pos}")




to_robot = Gf.Vec3d(robot_base_pos[0] - cluster_center[0],
                     robot_base_pos[1] - cluster_center[1], 0.0)
if to_robot.GetLength() < 1e-6:
    raise RuntimeError("Robot base is directly above/below cluster center - "
                        "horizontal forward direction is undefined, fix manually")
forward = to_robot.GetNormalized()
right = Gf.Cross(forward, WORLD_UP).GetNormalized()
up = Gf.Cross(right, forward).GetNormalized()
print(f"[seed={SEED}] Local frame - forward (toward robot): {forward}")




def sample_camera_pose():
   dist = random.uniform(*DIST_RANGE)
   el_deg = random.uniform(*ELEVATION_RANGE_DEG)

   az_deg = random.uniform(*AZIMUTH_RANGE_DEG)
   while AZIMUTH_DEADZONE_DEG[0] <= az_deg <= AZIMUTH_DEADZONE_DEG[1]:
       az_deg = random.uniform(*AZIMUTH_RANGE_DEG)

   az = math.radians(az_deg)
   el = math.radians(el_deg)

   offset = (dist * math.cos(el) * math.cos(az)) * forward \
            + (dist * math.cos(el) * math.sin(az)) * right \
            + (dist * math.sin(el)) * up
   cam_pos = cluster_center + offset

   view_matrix = Gf.Matrix4d().SetLookAt(cam_pos, cluster_center, WORLD_UP)
   cam_to_world = view_matrix.GetInverse()
   return cam_pos, cam_to_world, dist, az_deg, el_deg






def link6_target_from_camera_pose(cam_to_world):
    link6_to_world = CAMERA_LOCAL_TO_LINK6.GetInverse() * cam_to_world
    link6_pos = link6_to_world.ExtractTranslation()
    link6_rot = link6_to_world.ExtractRotationMatrix()
    return link6_pos, link6_rot, link6_to_world




print(f"\n{'#':<4}{'cam_pos':<40}{'dist':<8}{'link6_pos'}")
print("-" * 95)


results = []
for i in range(N_SAMPLES):
    cam_pos, cam_to_world, dist, az_deg, el_deg = sample_camera_pose()
    link6_pos, link6_rot, link6_to_world = link6_target_from_camera_pose(cam_to_world)


    reconstructed_cam_to_world = CAMERA_LOCAL_TO_LINK6 * link6_to_world
    position_error = (reconstructed_cam_to_world.ExtractTranslation() - cam_pos).GetLength()


    forward_axis = cam_to_world.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    reconstructed_forward = reconstructed_cam_to_world.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    orientation_error_deg = math.degrees(math.acos(
        max(-1.0, min(1.0, Gf.Dot(forward_axis, reconstructed_forward)))
    ))


    results.append({
        "cam_pos": cam_pos, "link6_pos": link6_pos, "link6_rot": link6_rot,
        "look_at_target": cluster_center,
        "sampled_dist": dist, "sampled_azimuth_deg": az_deg, "sampled_elevation_deg": el_deg,
        "position_check_error": position_error,
        "orientation_check_error_deg": orientation_error_deg,
    })


max_pos_error = max(r["position_check_error"] for r in results)
max_orient_error = max(r["orientation_check_error_deg"] for r in results)
print(f"\n[seed={SEED}] Max position sanity check error: {max_pos_error:.8f}")
print(f"[seed={SEED}] Max orientation sanity check error: {max_orient_error:.4f} deg")


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
    robot_marker.CreateDisplayColorAttr([(0.0, 0.0, 1.0)])
    print(f"\nDebug spheres at {DEBUG_ROOT} - yellow=candidate cams, red=cluster center, blue=robot base")



