"""
manipulability_check.py -- computes a real manipulability index for every
solved pose in ik_results_seed*.json, using MoveIt's /compute_fk service.

WHY THIS EXISTS:
compute_ik_client.py reports a pose "solved" as soon as ANY valid joint
configuration is found -- it says nothing about whether that
configuration is comfortably posed or sitting right on the edge of a
singularity (loses a degree of freedom, huge joint velocities needed for
tiny end-effector motion). That's exactly the failure mode reported from
real-hardware testing (arm extended, loses motion). This script measures
that directly instead of guessing a "safe" joint-angle range.

THIS SCRIPT DOES NOT FILTER/REJECT ANYTHING YET -- it only measures and
reports. Per this project's own repeated pattern (v8 SS3.2 trellis
tolerance, v13 SS1.6 drive tuning): look at the real distribution of
values first, THEN pick a threshold from real data, not a guess.
"""
import glob
import json
import os
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


IK_LINK_NAME = "Link6"       # same tip link compute_ik_client.py solves for
BASE_FRAME = "base_link"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

FD_EPSILON_RAD = 1e-4  # finite-difference perturbation size, central difference (+eps and -eps)


class FKClient(Node):
    def __init__(self):
        super().__init__("manipulability_fk_client")
        self.client = self.create_client(GetPositionFK, "/compute_fk")
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "/compute_fk service not available after 5s -- is "
                "moveit_demo.launch.py actually running?"
            )

    def get_link6_pose(self, joint_positions):
        """Returns (position_xyz, quat_xyzw) for Link6 at the given joint config."""
        req = GetPositionFK.Request()
        req.header = Header()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = [IK_LINK_NAME]

        rs = RobotState()
        js = JointState()
        js.name = JOINT_NAMES
        js.position = list(joint_positions)
        rs.joint_state = js
        req.robot_state = rs

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            raise RuntimeError("FK call timed out")

        resp = future.result()
        if resp.error_code.val != 1:
            raise RuntimeError(f"FK failed, error_code={resp.error_code.val}")

        pose = resp.pose_stamped[0].pose
        pos = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return pos, quat

    def numerical_jacobian(self, joint_positions):
        """
        Builds a 6x6 Jacobian by central-difference perturbation of each
        joint, one at a time: nudge joint i by +eps and -eps, ask FK
        where Link6 ends up each time, and
        (pose_at_plus - pose_at_minus) / (2*eps) is column i.

        Rows 0-2: linear velocity direction (position change).
        Rows 3-5: angular velocity direction (small-angle rotation vector,
        via scipy's axis-angle representation -- valid for small
        perturbations like FD_EPSILON_RAD).
        """
        J = np.zeros((6, 6))
        for i in range(6):
            plus = list(joint_positions)
            minus = list(joint_positions)
            plus[i] += FD_EPSILON_RAD
            minus[i] -= FD_EPSILON_RAD

            pos_plus, quat_plus = self.get_link6_pose(plus)
            pos_minus, quat_minus = self.get_link6_pose(minus)

            d_pos = (np.array(pos_plus) - np.array(pos_minus)) / (2 * FD_EPSILON_RAD)

            rot_plus = Rotation.from_quat(quat_plus)
            rot_minus = Rotation.from_quat(quat_minus)
            # relative rotation from "minus" pose to "plus" pose, as an
            # axis-angle vector -- this IS the angular displacement
            # direction*magnitude between the two perturbed poses.
            d_rot_vec = (rot_plus * rot_minus.inv()).as_rotvec() / (2 * FD_EPSILON_RAD)

            J[0:3, i] = d_pos
            J[3:6, i] = d_rot_vec
        return J


def manipulability_metrics(J):
    """
    Returns (manipulability_index, min_singular_value, condition_number).
    Singular values come from SVD -- numerically more stable than
    computing det(J) directly, and more informative since we also get
    the min singular value (the single worst direction) for free.
    """
    singular_values = np.linalg.svd(J, compute_uv=False)
    manipulability_index = float(np.prod(singular_values))
    min_sv = float(np.min(singular_values))
    max_sv = float(np.max(singular_values))
    condition_number = float(max_sv / min_sv) if min_sv > 1e-12 else float("inf")
    return manipulability_index, min_sv, condition_number


def check_all_results(result_paths):
    client = FKClient()
    all_rows = []

    for path in result_paths:
        with open(path) as f:
            entries = json.load(f)

        seed_tag = os.path.basename(path).replace("ik_results_", "").replace(".json", "")
        print(f"\n=== {seed_tag} ({len(entries)} solved poses) ===")
        print(f"{'#':<4}{'manip_index':<16}{'min_sing_val':<16}{'condition_#':<16}")
        print("-" * 60)

        for i, entry in enumerate(entries):
            if "manipulability_index" in entry:
                print(f"{i:<4}(already has manipulability data, skipping FK calls)")
                all_rows.append((path, entry))
                continue

            joints = entry["joint_positions"]
            J = client.numerical_jacobian(joints)
            manip_idx, min_sv, cond = manipulability_metrics(J)

            print(f"{i:<4}{manip_idx:<16.6f}{min_sv:<16.6f}"
                  f"{cond if cond != float('inf') else 'inf':<16}")

            row = dict(entry)
            row["manipulability_index"] = manip_idx
            row["min_singular_value"] = min_sv
            row["condition_number"] = cond
            all_rows.append((path, row))

    client.destroy_node()
    return all_rows





def print_summary(all_rows):
    manip_vals = [r["manipulability_index"] for _, r in all_rows]
    min_sv_vals = [r["min_singular_value"] for _, r in all_rows]

    print(f"\n=== SUMMARY across {len(all_rows)} solved poses ===")
    print(f"manipulability index -- min: {min(manip_vals):.6f}  max: {max(manip_vals):.6f}  "
          f"mean: {np.mean(manip_vals):.6f}  median: {np.median(manip_vals):.6f}")
    print(f"min singular value   -- min: {min(min_sv_vals):.6f}  max: {max(min_sv_vals):.6f}  "
          f"mean: {np.mean(min_sv_vals):.6f}  median: {np.median(min_sv_vals):.6f}")

    # No threshold applied here on purpose -- print the 5 worst poses so
    # we can look at real numbers together and decide where a sensible
    # cutoff actually sits, rather than guessing one in advance.
    worst = sorted(all_rows, key=lambda t: t[1]["min_singular_value"])[:5]
    print("\n5 poses closest to singular (lowest min singular value):")
    for seed_tag, r in worst:
        print(f"  [{seed_tag}] pos={tuple(round(p, 4) for p in r['position_xyz'])} "
              f"min_sv={r['min_singular_value']:.6f}  manip_idx={r['manipulability_index']:.6f}")


if __name__ == "__main__":
    _THIS_DIR = os.path.dirname(os.path.abspath(__file__))
    result_paths = sorted(
        p for p in glob.glob(os.path.join(_THIS_DIR, "ik_results_seed*.json"))
        if "_with_manip" not in os.path.basename(p)
    )

    if not result_paths:
        single_path = os.path.join(_THIS_DIR, "ik_results.json")
        if os.path.exists(single_path):
            result_paths = [single_path]
        else:
            raise RuntimeError("No ik_results*.json found -- run compute_ik_client.py first")

    rclpy.init()
    try:
        all_rows = check_all_results(result_paths)
        print_summary(all_rows)

        # Write manipulability data back INTO the original files, in place --
        # no new *_with_manip.json siblings. Safe to re-run: skips FK calls
        # for entries that already have manipulability_index (see check_all_results).
        by_path = {}
        for path, row in all_rows:
            by_path.setdefault(path, []).append(row)
        for path, rows in by_path.items():
            with open(path, "w") as f:
                json.dump(rows, f, indent=2)
            print(f"Updated {len(rows)} rows with manipulability data in {path}")
    finally:
        rclpy.shutdown()





