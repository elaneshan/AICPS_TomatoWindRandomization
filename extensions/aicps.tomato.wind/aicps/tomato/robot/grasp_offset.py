"""
grasp_offset.py -- fixed Link6 <-> grasp-point (gripper TCP) pose conversion.

"""
import numpy as np
from scipy.spatial.transform import Rotation


# --- THE ONE NUMBER THIS FILE EXISTS TO HOLD -------------------------------
# Distance in meters, measured along Link6's local +Z axis (= gripper's
# tool axis, post mount-fix -- confirmed aligned, no rotation offset, from Link6's origin out to the chosen grasp point.
#
# REAL, MEASURED VALUE -- via measure_grasp_offset.py,
# run live in Isaac Sim against real fingertip mesh geometry:
#   - Measured with the gripper closed to a 59.16mm gap (target: 60mm,
#     matching v8's ripe-tomato diameter target), at GRIPPER_TARGET_DEG
#     = -5.5 (found by iterative interpolation from -20deg -> -10deg ->
#     -5.5deg, since the crank linkage's gap-vs-angle relationship is
#     nonlinear -- confirmed nonlinear directly, not assumed: Z shifted
#     6.5mm over the -20->-10deg step but only 1.1mm over the -10->-5.5deg
#     step, consistent with the linkage flattening out near full closure).
#   - Local-frame X/Y both ~0 (order 1e-8 - 1e-7m) at every measurement,
#     confirming the fingers are symmetric about the tool axis as assumed.
GRASP_OFFSET_Z_M = 0.20575
# -----------------------------------------------------------------------


def _pose_to_matrix(position_xyz, quat_xyzw):
    """Position + quaternion -> 4x4 homogeneous transform matrix."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = position_xyz
    return T


def _matrix_to_pose(T):
    """4x4 homogeneous transform matrix -> (position_xyz, quat_xyzw)."""
    position_xyz = T[:3, 3].tolist()
    quat_xyzw = Rotation.from_matrix(T[:3, :3]).as_quat().tolist()
    return position_xyz, quat_xyzw


def _link6_to_grasp_transform(offset_m=GRASP_OFFSET_Z_M):
    """
    The transform FROM Link6's frame TO a point offset_m along Link6's
    own local Z axis. Defaults to GRASP_OFFSET_Z_M (the real, measured
    grasp point) -- pass a larger offset_m to get a standoff point
    further back along the same approach line instead.
    """
    T = np.eye(4)
    T[2, 3] = offset_m
    return T




def grasp_target_to_link6_target(grasp_position_xyz, grasp_quat_xyzw, offset_m=GRASP_OFFSET_Z_M):
    T_base_grasp = _pose_to_matrix(grasp_position_xyz, grasp_quat_xyzw)
    T_link6_grasp = _link6_to_grasp_transform(offset_m)
    T_base_link6 = T_base_grasp @ np.linalg.inv(T_link6_grasp)
    return _matrix_to_pose(T_base_link6)






def link6_pose_to_grasp_pose(link6_position_xyz, link6_quat_xyzw):
    """
    The reverse direction: given a Link6 pose (e.g. read back from an IK
    solution, or from real forward kinematics), return where the grasp
    point actually ended up. Useful for verification -- e.g. round-trip
    grasp_target_to_link6_target() -> solve IK -> forward-kinematics the
    result -> link6_pose_to_grasp_pose() -> should land back near your
    original grasp target, modulo whatever error IK itself introduced.
    """
    T_base_link6 = _pose_to_matrix(link6_position_xyz, link6_quat_xyzw)
    T_link6_grasp = _link6_to_grasp_transform()
    T_base_grasp = T_base_link6 @ T_link6_grasp
    return _matrix_to_pose(T_base_grasp)


if __name__ == "__main__":
    # Quick self-check: round-tripping grasp -> link6 -> grasp should
    # return (almost exactly) the original pose. This only validates the
    # ARITHMETIC is internally consistent -- same caveat as v9 SS4.1's
    # sampler self-check: it says nothing about whether GRASP_OFFSET_Z_M
    # itself is the RIGHT number, only that the matrix math is correct.
    test_grasp_pos = [0.3, 0.1, 0.25]
    test_grasp_quat = [0.0, 0.0, 0.0, 1.0]

    link6_pos, link6_quat = grasp_target_to_link6_target(test_grasp_pos, test_grasp_quat)
    back_pos, back_quat = link6_pose_to_grasp_pose(link6_pos, link6_quat)

    pos_error = np.linalg.norm(np.array(back_pos) - np.array(test_grasp_pos))
    print(f"grasp target:       pos={test_grasp_pos}  quat={test_grasp_quat}")
    print(f"-> link6 target:    pos={[round(p, 5) for p in link6_pos]}  quat={[round(q, 5) for q in link6_quat]}")
    print(f"-> back to grasp:   pos={[round(p, 5) for p in back_pos]}  quat={[round(q, 5) for q in back_quat]}")
    print(f"round-trip position error: {pos_error:.2e} m  (should be ~0)")


