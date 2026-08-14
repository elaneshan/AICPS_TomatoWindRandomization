"""
wrist_camera_lookat.py -- keeps the gripper-mounted wrist camera (EyeInHand_Camera)
dynamically aimed at the tomato cluster centroid as the arm moves, using
Gf.Matrix4d().SetLookAt()


USAGE:
    import aicps.tomato.robot.wrist_camera_lookat as wrist_camera_lookat
    wrist_camera_lookat.start_camera_lookat()   # call once per session
    wrist_camera_lookat.verify_aim()            # optional: recheck the view angle numerically
    ...
    wrist_camera_lookat.stop_camera_lookat()
"""

import numpy as np
import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, Gf

# Confirmed paths as of 2026-08-14 -- re-verify if the robot/cluster are
# ever re-imported or re-referenced (per this project's standing lesson:
# paths are not guaranteed stable across re-imports).
CAMERA_PATH = "/World/cr3/Geometry/world/dummy_link/base_link/Link1/Link2/Link3/Link4/Link5/Link6/Gripper/Geometry/gripper_base_link/EyeInHand_Camera"
CLUSTER_PATH = "/World/Cluster/Tomato_Cluster/Tomato_Cluster_Assembly"

UP_VECTOR = Gf.Vec3d(0, 0, 1)  # Z-up, consistent with the rest of this project

_camera_prim = None
_parent_prim = None
_local_offset = None
_cluster_center = None
_update_sub = None
_ready = False


def _current_local_translation(prim):
    """Reads the prim's CURRENT local translation relative to its parent
    from whatever is already authored on the stage -- preserves existing
    placement instead of guessing/hardcoding a number."""
    xform_cache = UsdGeom.XformCache()
    local_to_world = xform_cache.GetLocalToWorldTransform(prim)
    parent_to_world = xform_cache.GetLocalToWorldTransform(prim.GetParent())
    local = local_to_world * parent_to_world.GetInverse()
    return local.ExtractTranslation()


def _on_update(e):
    if _camera_prim is None or _parent_prim is None:
        return

    xform_cache = UsdGeom.XformCache()
    parent_world = xform_cache.GetLocalToWorldTransform(_parent_prim)

    # Position: fixed local offset, carried rigidly with the parent (arm) --
    # we are NOT moving the camera, only re-aiming it every frame.
    cam_world_pos = parent_world.Transform(_local_offset)

    view_matrix = Gf.Matrix4d()
    view_matrix.SetLookAt(Gf.Vec3d(cam_world_pos), Gf.Vec3d(_cluster_center), UP_VECTOR)
    cam_to_world = view_matrix.GetInverse()

    # USD is row-vector: World = Local * ParentWorld, so Local = World * ParentWorld^-1
    local_xform = cam_to_world * parent_world.GetInverse()

    xformable = UsdGeom.Xformable(_camera_prim)
    ops = xformable.GetOrderedXformOps()
    matrix_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
            matrix_op = op
            break
    if matrix_op is None:
        # one-time setup: collapse to a single matrix op, to avoid any
        # translate/orient op-ordering ambiguity on subsequent frames
        xformable.ClearXformOpOrder()
        matrix_op = xformable.AddTransformOp()
    matrix_op.Set(local_xform)


def start_camera_lookat():
    """Call once per session. Safe to call again -- stops any existing loop
    first and rebuilds fresh from the current live stage."""
    global _camera_prim, _parent_prim, _local_offset, _cluster_center, _update_sub, _ready

    if _update_sub is not None:
        stop_camera_lookat()

    stage = omni.usd.get_context().get_stage()

    _camera_prim = stage.GetPrimAtPath(CAMERA_PATH)
    if not _camera_prim.IsValid():
        raise RuntimeError(
            f"wrist_camera_lookat: no valid camera at {CAMERA_PATH} -- "
            f"check the path hasn't changed (re-imports can shift paths)."
        )
    _parent_prim = _camera_prim.GetParent()

    _local_offset = _current_local_translation(_camera_prim)
    print(f"wrist_camera_lookat: preserving current local offset: {_local_offset}")

    cluster_prim = stage.GetPrimAtPath(CLUSTER_PATH)
    if not cluster_prim.IsValid():
        raise RuntimeError(f"wrist_camera_lookat: no valid cluster at {CLUSTER_PATH}")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    _cluster_center = bbox_cache.ComputeWorldBound(cluster_prim).ComputeCentroid()
    print(f"wrist_camera_lookat: cluster centroid (world): {_cluster_center}")

    stream = omni.kit.app.get_app().get_update_event_stream()
    _update_sub = stream.create_subscription_to_pop(_on_update, name="wrist_camera_lookat")
    _ready = True
    print("wrist_camera_lookat: started. Camera will now track the cluster every frame.")


def stop_camera_lookat():
    global _update_sub, _ready
    if _update_sub is not None:
        _update_sub.unsubscribe()
        _update_sub = None
        _ready = False
        print("wrist_camera_lookat: stopped.")
    else:
        print("wrist_camera_lookat: was not running.")


def is_ready():
    return _ready


def verify_aim():
    """Re-measures the same view-angle metric used to diagnose the original
    50.34 deg aim problem -- confirms the fix numerically, not just visually.
    Should read close to 0 deg while the loop is running."""
    if not _ready:
        print("wrist_camera_lookat: not running -- call start_camera_lookat() first.")
        return
    xform_cache = UsdGeom.XformCache()
    cam_world = xform_cache.GetLocalToWorldTransform(_camera_prim)
    cam_pos = cam_world.ExtractTranslation()
    cam_forward = cam_world.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    to_cluster = (_cluster_center - cam_pos).GetNormalized()
    dot = max(-1.0, min(1.0, cam_forward * to_cluster))
    angle_deg = np.degrees(np.arccos(dot))
    print(f"wrist_camera_lookat: current view angle = {angle_deg:.2f} deg (target: ~0)")
    return angle_deg