"""
export_sampled_poses.py -- run this in the Kit Script Editor.

Loads camera_pose_sampler.py (needs omni.usd/pxr, only available in Kit's
Python), converts its results to plain JSON-serializable tuples, and writes
them to disk. A separate process running system python3 (which has the real
rclpy bindings -- see compute_ik_client.py's __main__) reads this file.

Why two processes: Kit's embedded Python is 3.12; rclpy's compiled C
extension (_rclpy_pybind11) only exists for ROS Humble's Python 3.10.
Confirmed via direct test -- `import rclpy` inside the Kit script editor
fails with ModuleNotFoundError on the .so itself, not a path problem, so
there is no in-process fix. Running the IK step as a separate system-python3
process (already confirmed working: 2/2 poses solved via `python3
compute_ik_client.py`) is the actual fix, not a workaround to revisit later.
"""
import sys
import json
import importlib.util

ROBOT_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/robot"
# NOT /tmp -- Kit's process and the host terminal may not share a /tmp
# (e.g. if Isaac Sim runs in a container/different mount namespace). Using
# the project folder itself is a location both sides have already
# confirmed they can reach (compute_ik_client.py already ran successfully
# from this exact directory via a plain terminal).
OUTPUT_PATH = f"{ROBOT_PKG_DIR}/sampled_poses.json"

if ROBOT_PKG_DIR not in sys.path:
    sys.path.insert(0, ROBOT_PKG_DIR)


def _load_module(name, filename):
    path = f"{ROBOT_PKG_DIR}/{filename}"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def gf_rotation_matrix_to_quat_xyzw(rot_matrix3d):
    """link6_rot from the sampler is a Gf.Matrix3d. ROS/MoveIt need a plain
    (x, y, z, w) quaternion. NOTE: not yet independently verified that USD's
    Gf.Quatd imaginary/real ordering matches ROS's xyzw convention -- if
    solved poses look reachable but the robot ends up mis-oriented once
    executed, check this conversion first."""
    rotation = rot_matrix3d.ExtractRotation()  # Gf.Rotation
    quat = rotation.GetQuat()  # Gf.Quatd
    imag = quat.GetImaginary()
    return [imag[0], imag[1], imag[2], quat.GetReal()]


# camera_pose_sampler.py runs its full sampling loop at import time and
# populates module-level `results` (list of dicts: cam_pos, link6_pos,
# link6_rot, offset_check_error) -- same as it's been used all session.
sampler = _load_module("camera_pose_sampler", "camera_pose_sampler.py")

# --- WORLD -> base_link transform ---
# The sampler's link6_pos/link6_rot are in Isaac Sim's /World frame. MoveIt's
# compute_ik request needs the target pose in the planning frame, which per
# the SRDF chain (base_link -> Link6) and compute_ik_client.py's BASE_FRAME
# is "base_link". Sending world-frame numbers directly under a "base_link"
# frame_id silently asks MoveIt to reach ~0.9m away instead of the real
# ~0.15-0.45m -- this was the actual cause of the 10/10 NO_IK_SOLUTION
# (error_code=-31) failures, confirmed by checking the raw distance from
# robot_base_pos to the sampled points before this fix, not guessed.
from pxr import Usd, UsdGeom

base_to_world = UsdGeom.Xformable(sampler.robot_base_prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()
)
world_to_base = base_to_world.GetInverse()
world_to_base_rot = world_to_base.ExtractRotationMatrix()

exportable = []
PRACTICAL_REACH_M = 0.62   # confirmed spec, v7 5.1 -- conservative, real-world envelope
RAW_KINEMATIC_MAX_M = 0.807  # v7 5.1's own grid/random search over legal joint
                              # angles found this as the extreme-diagonal-pose max.
                              # NOT to be used as a target design distance (v7 was
                              # explicit about this) -- but it does mean distances
                              # between the two numbers below are a real gray zone,
                              # not simply "unreachable". Confirmed this session:
                              # some >0.62m samples solved fine, some didn't --
                              # orientation/joint-config dependent, not distance alone.

print(f"\n{'#':<4}{'local_pos (base_link frame)':<40}{'dist_from_base':<18}{'classification'}")
print("-" * 95)

n_plausible = n_borderline = n_implausible = 0
for i, r in enumerate(sampler.results):
    local_pos = world_to_base.Transform(r["link6_pos"])
    local_rot = world_to_base_rot * r["link6_rot"]
    dist_from_base = local_pos.GetLength()
    if dist_from_base < PRACTICAL_REACH_M:
        classification = "PLAUSIBLE"
        n_plausible += 1
    elif dist_from_base < RAW_KINEMATIC_MAX_M:
        classification = "BORDERLINE (orientation-dependent, worth trying)"
        n_borderline += 1
    else:
        classification = "IMPLAUSIBLE"
        n_implausible += 1
    print(f"{i:<4}{str(tuple(round(v, 4) for v in local_pos)):<40}"
          f"{dist_from_base:<18.4f}{classification}")
    exportable.append({
        "position_xyz": [local_pos[0], local_pos[1], local_pos[2]],
        "quat_xyzw": gf_rotation_matrix_to_quat_xyzw(local_rot),
        "dist_from_base": dist_from_base,
        "reach_classification": classification,
    })

print(f"\n{n_plausible} plausible / {n_borderline} borderline / {n_implausible} implausible "
      f"out of {len(exportable)} samples.")
if n_plausible + n_borderline == 0:
    print("WARNING: 0 plausible or borderline samples -- expect compute_ik to fail on "
          "all of these. Fix the sampling range or move the cluster/robot closer "
          "before spending IK round-trips on this batch.")

with open(OUTPUT_PATH, "w") as f:
    json.dump(exportable, f, indent=2)

import os
abs_path = os.path.abspath(OUTPUT_PATH)
exists = os.path.exists(OUTPUT_PATH)
print(f"Wrote {len(exportable)} sampled poses to {OUTPUT_PATH}")
print(f"Absolute path (confirm this matches what the terminal sees): {abs_path}")
print(f"os.path.exists() right after write: {exists}")
print("Now run in a ROS2-sourced terminal (system python3, NOT Kit's):")
print(f"  cd {ROBOT_PKG_DIR}")
print("  python3 compute_ik_client.py")


