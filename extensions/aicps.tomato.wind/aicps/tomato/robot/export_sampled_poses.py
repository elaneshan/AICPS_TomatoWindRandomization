"""
export_sampled_poses.py 

UPDATED: runs camera_pose_sampler_v2.py once per seed in SAMPLER_SEEDS,
via the SAMPLER_SEED env var (see that file's SEED line), and writes one
sampled_poses_seed{N}.json per batch. compute_ik_client.py's multi-batch
mode auto-detects files matching this naming and reports per-batch plus
aggregate success rates.

Each call to _load_module() re-executes the sampler script fresh (it's
loaded via importlib.util.spec_from_file_location + exec_module, not a
cached import), so setting SAMPLER_SEED and reloading between iterations
correctly produces independent, reproducible batches -- no stale state
carries over between seeds.
"""
import os
import sys
import json
import importlib.util
import math
import omni.usd as usd
from pxr import UsdGeom, Gf, Usd


ROBOT_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/robot"
# NOT /tmp -- Kit's process and the host terminal may not share a /tmp.

SAMPLER_SEEDS = [42, 43, 44]  # edit this list to change which/how many batches run

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
    rotation = rot_matrix3d.ExtractRotation()
    quat = rotation.GetQuat()
    imag = quat.GetImaginary()
    return [imag[0], imag[1], imag[2], quat.GetReal()]


PRACTICAL_REACH_M = 0.62
RAW_KINEMATIC_MAX_M = 0.807


def export_one_seed(seed):
    os.environ["SAMPLER_SEED"] = str(seed)
    sampler = _load_module(f"camera_pose_sampler_seed{seed}", "camera_pose_sampler.py")

    base_to_world = UsdGeom.Xformable(sampler.robot_base_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    world_to_base = base_to_world.GetInverse()
    world_to_base_rot = world_to_base.ExtractRotationMatrix()

    # Real camera-mount rotation, same one used inside the sampler --
    # needed here too, since local_rot below is Link6's orientation,
    # not the camera's, and only camera orientation is meaningful to
    # check against the look-at target.
    camera_mount_rot = sampler.CAMERA_LOCAL_TO_LINK6.ExtractRotationMatrix()

    exportable = []
    n_plausible = n_borderline = n_implausible = 0

    print(f"\n{'#':<4}{'local_pos (base_link frame)':<40}{'dist_from_base':<18}{'classification'}")
    print("-" * 95)

    for i, r in enumerate(sampler.results):
        local_pos = world_to_base.Transform(r["link6_pos"])
        local_rot = r["link6_rot"] * world_to_base_rot
        local_target = world_to_base.Transform(r["look_at_target"])
        local_cam_pos = world_to_base.Transform(r["cam_pos"])  # camera's real position, not Link6's

        # Real camera orientation-in-base-frame = mount rotation composed
        # with Link6's base-frame rotation. Checking local_rot's own -Z
        # directly (as before) tests Link6's own facing direction, which
        # is NOT the camera's viewing axis once the mount carries a real
        # rotation (confirmed: mount deviation from identity = 4.0565,
        # not ~0) -- that mismatch is exactly what produced the
        # ~154-157 deg readings.
        camera_rot_in_base = camera_mount_rot * local_rot
        camera_rot_in_base_4 = Gf.Matrix4d(camera_rot_in_base, Gf.Vec3d(0, 0, 0))
        forward_axis = camera_rot_in_base_4.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
        intended_dir = (local_target - local_cam_pos).GetNormalized()
        orientation_error_deg = math.degrees(math.acos(
            max(-1.0, min(1.0, Gf.Dot(forward_axis, intended_dir)))
        ))
        print(f"  [{i}] orientation check error: {orientation_error_deg:.2f} deg")

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
            "look_at_target_xyz": [local_target[0], local_target[1], local_target[2]],  # NEW
            "sampled_dist": r["sampled_dist"],                # NEW
            "sampled_azimuth_deg": r["sampled_azimuth_deg"],
            "sampled_elevation_deg": r["sampled_elevation_deg"],
        })



    print(f"\n[seed={seed}] {n_plausible} plausible / {n_borderline} borderline / "
          f"{n_implausible} implausible out of {len(exportable)} samples.")
    if n_plausible + n_borderline == 0:
        print(f"WARNING [seed={seed}]: 0 plausible or borderline samples.")

    out_path = f"{ROBOT_PKG_DIR}/sampled_poses_seed{seed}.json"
    with open(out_path, "w") as f:
        json.dump(exportable, f, indent=2)
    print(f"[seed={seed}] Wrote {len(exportable)} sampled poses to {out_path}")
    return out_path


if __name__ == "__main__":
    written = []
    for seed in SAMPLER_SEEDS:
        written.append(export_one_seed(seed))

    print(f"\n=== Wrote {len(written)} batches ===")
    for p in written:
        print(f"  {p}")
    print("\nNow run in a ROS2-sourced terminal (system python3, NOT Kit's):")
    print(f"  cd {ROBOT_PKG_DIR}")
    print("  python3 compute_ik_client.py")
    print("(it auto-detects sampled_poses_seed*.json and reports per-batch + aggregate)")

