"""
manipulation_targets.py -- samples a grasp target for a manipulation
episode: picks a random pedicel or leaf from the rig, and builds a full
grasp pose (position + orientation) at its hinge point.

currently defaults to a fixed gripper grasp orientation, but should eventually be able to sample a range of orientations based on the target's branch orientation
"""
import random
from pxr import Gf, UsdGeom, Usd

import grasp_offset


WORLD_UP = Gf.Vec3d(0, 0, 1)

STANDOFF_FRACTION = 0.23   
STANDOFF_MIN_M = 0.12
STANDOFF_MAX_M = 0.25




def sample_standoff_target(rig, robot_base_prim, gripper_target_deg=-5.5):
    candidates = list(rig.pedicels) + list(rig.leaves)
    if not candidates:
        return None
    item = random.choice(candidates)


    robot_base_pos = UsdGeom.Xformable(robot_base_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    ).ExtractTranslation()


    grasp_pos = [item.hinge_point[0], item.hinge_point[1], item.hinge_point[2]]
    grasp_quat = _approach_orientation(item.hinge_point, robot_base_pos)


    grasp_dist_from_base = (Gf.Vec3d(*grasp_pos) - robot_base_pos).GetLength()
    standoff_offset_m = max(STANDOFF_MIN_M, min(STANDOFF_MAX_M, grasp_dist_from_base * STANDOFF_FRACTION))


    link6_pos, link6_quat = grasp_offset.grasp_target_to_link6_target(
        grasp_pos, grasp_quat, offset_m=standoff_offset_m
    )


    return {
        "target_prim_name": item.prim.GetName(),
        "target_type": "pedicel" if item in rig.pedicels else "leaf",
        "grasp_position_xyz": grasp_pos,
        "grasp_quat_xyzw": grasp_quat,
        "position_xyz": link6_pos,
        "quat_xyzw": link6_quat,
        "gripper_target_deg": gripper_target_deg,
        "is_standoff": True,
        "standoff_offset_m": standoff_offset_m,
    }









def _approach_orientation(target_pos, robot_base_pos):
    """
    Builds a quaternion (x,y,z,w) for the grasp point's orientation:
    gripper's approach axis (local -Z, same convention as the camera's
    look-at) points from the target back toward the robot base -- i.e.
    the gripper approaches the target FROM the robot's own direction,
    the only geometrically sane default without real branch-orientation
    data.
    """
    view_matrix = Gf.Matrix4d().SetLookAt(target_pos, robot_base_pos, WORLD_UP)
    cam_to_world = view_matrix.GetInverse()
    rot = cam_to_world.ExtractRotation().GetQuat()
    imag = rot.GetImaginary()
    return [imag[0], imag[1], imag[2], rot.GetReal()]


def sample_grasp_target(rig, robot_base_prim, gripper_target_deg=-5.5):
    """
    Picks a random pedicel or leaf from rig.pedicels/rig.leaves, builds
    a grasp target pose at its hinge_point, and converts it to a Link6
    target via grasp_offset.py.

    Returns a dict matching the same shape sample_target() callers
    expect elsewhere in this project (position_xyz/quat_xyzw are the
    LINK6 target, ready for solve_ik -- grasp_position_xyz/
    grasp_quat_xyzw are kept too, for metadata/debugging).
    Returns None if rig has no pedicels or leaves at all (empty rig --
    should not happen in practice, but don't silently crash on it).
    """
    candidates = list(rig.pedicels) + list(rig.leaves)
    if not candidates:
        return None
    item = random.choice(candidates)

    robot_base_pos = UsdGeom.Xformable(robot_base_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    ).ExtractTranslation()

    grasp_pos = [item.hinge_point[0], item.hinge_point[1], item.hinge_point[2]]
    grasp_quat = _approach_orientation(item.hinge_point, robot_base_pos)

    link6_pos, link6_quat = grasp_offset.grasp_target_to_link6_target(grasp_pos, grasp_quat)

    return {
        "target_prim_name": item.prim.GetName(),
        "target_type": "pedicel" if item in rig.pedicels else "leaf",
        "grasp_position_xyz": grasp_pos,
        "grasp_quat_xyzw": grasp_quat,
        "position_xyz": link6_pos,   # what solve_ik actually consumes
        "quat_xyzw": link6_quat,
        "gripper_target_deg": gripper_target_deg,
    }


