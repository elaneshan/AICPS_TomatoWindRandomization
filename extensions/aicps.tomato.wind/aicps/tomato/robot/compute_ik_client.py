"""
compute_ik_client.py -- calls MoveIt's existing /compute_ik service
(moveit_msgs/srv/GetPositionIK) with Cartesian Link6 target poses produced
by the v2 camera-pose sampler (this version): multi-seed IK retry + multi-batch support.

"""
import glob
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import random

GROUP_NAME = "cr3_group"
IK_LINK_NAME = "Link6"
BASE_FRAME = "base_link"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

IK_TIMEOUT_SEC = 0.5  # per-request timeout passed IN the request

# --- Candidate seeds, tried in order per pose until one solves ---
ZERO_SEED = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOME_SEED = [0.0, 0.6072, -1.7223, -0.2949, 1.6134, 0.0]  # SRDF group_state "home"

# Third fallback: a random draw within a conservative range, regenerated
# fresh per pose (not fixed), on the theory that if neither zero nor home
# is a good starting basin for a given pose, a random third guess has a
# decent chance of landing in a different, better one. +-2.5 rad is
# comfortably inside typical joint ranges without needing joint_limits.yaml
# hard-coded here; if this keeps failing alongside the first two, that's a
# real signal worth checking joint_limits.yaml directly rather than
# widening this blindly.
RANDOM_SEED_RANGE = 2.5  # radians, +/-


def random_seed_guess():
    return [random.uniform(-RANDOM_SEED_RANGE, RANDOM_SEED_RANGE) for _ in range(6)]


SEED_LABELS_BASE = ["zero", "home"]  # random seed gets a per-attempt label appended


class ComputeIKClient(Node):
    def __init__(self):
        super().__init__("compute_ik_client")
        self.client = self.create_client(GetPositionIK, "/compute_ik")
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "/compute_ik service not available after 5s -- is "
                "moveit_demo.launch.py actually running?"
            )

    def _solve_once(self, position_xyz, quat_xyzw, seed_joint_positions):
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = GROUP_NAME
        ik_req.ik_link_name = IK_LINK_NAME
        ik_req.timeout = Duration(seconds=IK_TIMEOUT_SEC).to_msg()
        ik_req.avoid_collisions = False  # planning-scene collision objects
        # not registered yet -- deliberately deferred until after basic IK
        # is proven out.

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = float(position_xyz[0])
        pose.pose.position.y = float(position_xyz[1])
        pose.pose.position.z = float(position_xyz[2])
        pose.pose.orientation.x = float(quat_xyzw[0])
        pose.pose.orientation.y = float(quat_xyzw[1])
        pose.pose.orientation.z = float(quat_xyzw[2])
        pose.pose.orientation.w = float(quat_xyzw[3])
        ik_req.pose_stamped = pose

        seed = RobotState()
        js = JointState()
        js.name = JOINT_NAMES
        js.position = list(seed_joint_positions)
        seed.joint_state = js
        ik_req.robot_state = seed

        req.ik_request = ik_req

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=IK_TIMEOUT_SEC + 2.0)

        if not future.done():
            return False, None, None

        resp = future.result()
        success = resp.error_code.val == 1
        if success:
            name_to_pos = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
            joint_positions = [name_to_pos[j] for j in JOINT_NAMES]
        else:
            joint_positions = None
        return success, joint_positions, resp.error_code.val

    def solve_multi_seed(self, position_xyz, quat_xyzw, n_random_fallbacks=1):
        """
        Tries ZERO_SEED, then HOME_SEED, then n_random_fallbacks fresh
        random seeds, in that order, returning as soon as one succeeds.

        Returns (success, joint_positions, error_code, seed_label,
                 attempts_made) -- seed_label tells you WHICH seed solved
        it (useful for spotting patterns like sample 2's seed-sensitivity
        across future batches), attempts_made tells you how many tries
        it took even on success.
        """
        candidates = [("zero", ZERO_SEED), ("home", HOME_SEED)]
        for i in range(n_random_fallbacks):
            candidates.append((f"random_{i}", random_seed_guess()))

        last_err_code = None
        for attempt_num, (label, seed_vals) in enumerate(candidates, start=1):
            success, joints, err_code = self._solve_once(position_xyz, quat_xyzw, seed_vals)
            last_err_code = err_code
            if success:
                return True, joints, err_code, label, attempt_num
        return False, None, last_err_code, None, len(candidates)


def run_batch(sampled_poses, n_random_fallbacks=1, batch_label=""):
    client = ComputeIKClient()
    results = []
    error_code_counts = {}
    seed_label_counts = {}

    for i, entry in enumerate(sampled_poses):
        pos = tuple(entry["position_xyz"])
        quat = tuple(entry["quat_xyzw"])
        success, joints, err_code, seed_label, attempts = client.solve_multi_seed(
            pos, quat, n_random_fallbacks=n_random_fallbacks
        )
        error_code_counts[err_code] = error_code_counts.get(err_code, 0) + 1
        if success:
            seed_label_counts[seed_label] = seed_label_counts.get(seed_label, 0) + 1
            status = f"OK (seed={seed_label}, attempt {attempts})"
            results.append((entry, joints, seed_label))
        else:
            status = f"FAIL after {attempts} seed attempts (last error_code={err_code})"
        print(f"[{batch_label}{i:03d}] pos={tuple(round(p, 4) for p in pos)} -> {status}")
    client.destroy_node()

    n = len(sampled_poses)
    n_ok = len(results)
    print(f"\n--- {batch_label or 'batch'} summary ---")
    print(f"{n_ok}/{n} poses solved successfully ({100 * n_ok / n:.1f}%)")
    print(f"error_code breakdown (per attempt, not per pose): {error_code_counts}")
    print(f"which seed solved it (successes only): {seed_label_counts}")
    return results


def load_sampled_poses(path):
    import json
    with open(path) as f:
        return json.load(f)  # full entries now, not just (pos, quat) tuples


def save_results(results, path):
    import json
    out = []
    for entry, joints, label in results:
        merged = dict(entry)  # preserves dist_from_base, reach_classification,
                               # look_at_target_xyz, sampled_dist/azimuth/elevation
        merged["joint_positions"] = list(joints)
        merged["seed_used"] = label
        out.append(merged)
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"Wrote {len(out)} solved poses to {path}")



if __name__ == "__main__":
    _THIS_DIR = os.path.dirname(os.path.abspath(__file__))

    # Picks up sampled_poses.json (single-batch, old naming) if present,
    # otherwise looks for sampled_poses_seed*.json (multi-batch naming --
    # see export_sampled_poses.py's SEED-suffixed output). Edit
    # BATCH_PATTERN directly if your filenames differ.
    single_path = os.path.join(_THIS_DIR, "sampled_poses.json")
    multi_paths = sorted(glob.glob(os.path.join(_THIS_DIR, "sampled_poses_seed*.json")))

    rclpy.init()
    try:
        all_results = []
        if multi_paths:
            print(f"Found {len(multi_paths)} seed batches: {[os.path.basename(p) for p in multi_paths]}\n")
            for path in multi_paths:
                seed_tag = os.path.basename(path).replace("sampled_poses_", "").replace(".json", "")
                poses = load_sampled_poses(path)
                print(f"=== {seed_tag} ({len(poses)} poses) ===")
                results = run_batch(poses, batch_label=f"{seed_tag}_")
                all_results.extend(results)
                if results:
                    out_path = os.path.join(_THIS_DIR, f"ik_results_{seed_tag}.json")
                    save_results(results, out_path)
                print()
        elif os.path.exists(single_path):
            poses = load_sampled_poses(single_path)
            print(f"Loaded {len(poses)} sampled poses from {single_path}")
            all_results = run_batch(poses)
            if all_results:
                save_results(all_results, os.path.join(_THIS_DIR, "ik_results.json"))
        else:
            print("No sampled-poses file(s) found. Run export_sampled_poses.py first.")
            print("Falling back to a 2-pose smoke test:")
            example_poses = [
                ((0.3, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
                ((0.35, 0.1, 0.25), (0.0, 0.0, 0.0, 1.0)),
            ]
            all_results = run_batch(example_poses)

        if multi_paths:
            n_total_poses = sum(len(load_sampled_poses(p)) for p in multi_paths)
            print(f"\n=== AGGREGATE across {len(multi_paths)} sampler-seed batches ===")
            print(f"{len(all_results)}/{n_total_poses} poses solved "
                  f"({100 * len(all_results) / n_total_poses:.1f}%)")
            print("(compare against: 90.0% single-batch result with home-seed-only + "
                  "TRAC-IK -- this aggregate tells you whether that held up across "
                  "different sampled poses, or was specific to that one batch)")
    finally:
        rclpy.shutdown()

