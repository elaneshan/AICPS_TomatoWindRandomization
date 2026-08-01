"""
compute_ik_client.py -- calls MoveIt's existing /compute_ik service
(moveit_msgs/srv/GetPositionIK) with Cartesian Link6 target poses produced
by the v2 camera-pose sampler (v9 doc, section 4.3).

This does NOT build a new IK solver. It calls the one already running as
part of the PI's moveit_demo.launch.py / move_group node, confirmed via:
    ros2 service list | grep compute_ik   -> /compute_ik
    ros2 service type /compute_ik         -> moveit_msgs/srv/GetPositionIK

Confirmed from cr3_robot.srdf:
    group_name  = "cr3_group"
    ik_link_name = "Link6"   (chain base_link -> Link6, matches the sampler's
                               target frame exactly, no reprojection needed)

ASSUMPTIONS -- check these against your real setup before trusting results:
    - frame_id for the target pose is "base_link". This is the SRDF chain's
      base link and is very likely also the planning frame, but this has NOT
      been independently confirmed (e.g. via `ros2 topic echo /tf` or the
      MoveIt planning frame param). If IK calls fail 100% of the time even
      for poses you know are reachable, check this first -- a frame mismatch
      is a classic silent-failure cause here.
    - kinematics.yaml's solver timeout (0.005s) is tight. This script passes
      its own more generous per-request timeout, but if you see a high
      failure rate, don't assume "unreachable" until you've checked whether
      it's actually solver timeout (see run_batch()'s summary output, which
      separates these two possibilities where the service response allows it).

Usage pattern (per v9 doc section 5.2 validation plan):
    - run this against N sampled poses from the v2 sampler
    - compare success-rate against the old baseline (10.5% plausible / 0%
      well-aimed from the pure random-joint-sampling sweep, v9 section 2.2)
    - a failure here means "IK-unreachable," a fundamentally different and
      more informative signal than the old "badly aimed" failure mode
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

GROUP_NAME = "cr3_group"
IK_LINK_NAME = "Link6"
BASE_FRAME = "base_link"  # ASSUMPTION -- see module docstring, confirm this
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# per-request timeout passed IN the request (independent of kinematics.yaml's
# own 0.005s solver-internal timeout) -- generous on purpose while validating
IK_TIMEOUT_SEC = 0.5


class ComputeIKClient(Node):
    def __init__(self):
        super().__init__("compute_ik_client")
        self.client = self.create_client(GetPositionIK, "/compute_ik")
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "/compute_ik service not available after 5s -- is "
                "moveit_demo.launch.py actually running?"
            )

    def solve(self, position_xyz, quat_xyzw, seed_joint_positions=None):
        """
        position_xyz: (x, y, z) in BASE_FRAME, meters
        quat_xyzw: (x, y, z, w)
        seed_joint_positions: optional list[6] of floats, radians, in
            JOINT_NAMES order. If None, seeds with all-zero (matches the
            robot's current sim default per v8 -- home-position calibration
            is still pending PI's real-world measurement, per v8 section 5).

        Returns (success: bool, joint_positions: list[float] | None,
                 error_code: int, raw_response)
        """
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = GROUP_NAME
        ik_req.ik_link_name = IK_LINK_NAME
        ik_req.timeout = Duration(seconds=IK_TIMEOUT_SEC).to_msg()
        ik_req.avoid_collisions = False  # no planning-scene collision objects
        # registered yet -- v9 section 4.4 / section 5.1 explicitly defers
        # MoveIt planning-scene collision registration until after basic IK
        # is proven out. Setting True right now would not check anything
        # meaningful (empty scene) and could mask config problems, so leave
        # False until that work is actually done.

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
        js.position = list(seed_joint_positions) if seed_joint_positions else [0.0] * 6
        seed.joint_state = js
        ik_req.robot_state = seed

        req.ik_request = ik_req

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=IK_TIMEOUT_SEC + 2.0)

        if not future.done():
            self.get_logger().warn("compute_ik call did not complete (node-level timeout)")
            return False, None, None, None

        resp = future.result()
        # error_code.val == 1 is moveit_msgs/MoveItErrorCodes.SUCCESS
        success = resp.error_code.val == 1
        if success:
            name_to_pos = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
            joint_positions = [name_to_pos[j] for j in JOINT_NAMES]
        else:
            joint_positions = None

        return success, joint_positions, resp.error_code.val, resp


def run_batch(sampled_poses, seed_joint_positions=None):
    """
    sampled_poses: list of (position_xyz, quat_xyzw) tuples -- e.g. the
        (link6_pos, link6_rot) output of the v2 camera-pose sampler,
        converted to plain (x,y,z) / (x,y,z,w) tuples before calling this.

    Prints a summary (mirrors the style of the v9 section 2.2 reach-sweep
    summary, for direct before/after comparison) and returns the list of
    successful (position, quat, joint_positions) results.
    """
    rclpy.init()
    client = ComputeIKClient()
    results = []
    error_code_counts = {}

    try:
        for i, (pos, quat) in enumerate(sampled_poses):
            success, joints, err_code, _ = client.solve(pos, quat, seed_joint_positions)
            error_code_counts[err_code] = error_code_counts.get(err_code, 0) + 1
            status = "OK" if success else f"FAIL (error_code={err_code})"
            print(f"[{i:03d}] pos={tuple(round(p, 4) for p in pos)} -> {status}")
            if success:
                results.append((pos, quat, joints))
    finally:
        client.destroy_node()
        rclpy.shutdown()

    n = len(sampled_poses)
    n_ok = len(results)
    print(f"\n{n_ok}/{n} poses solved successfully ({100 * n_ok / n:.1f}%)")
    print(f"error_code breakdown: {error_code_counts}")
    print(
        "(compare against v9 section 2.2 baseline: 10.5% plausible / 0% "
        "close+well-aimed under pure random joint sampling -- this number "
        "should be substantially higher, since poses are now deliberately "
        "aimed and only IK-reachability, not aim, should be limiting it)"
    )
    return results


import os as _os
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
# Matches export_sampled_poses.py's OUTPUT_PATH -- both scripts write/read
# from the project folder itself, not /tmp (Kit's process and the host
# terminal may not share a /tmp, e.g. under a container).
SAMPLED_POSES_PATH = _os.path.join(_THIS_DIR, "sampled_poses.json")
IK_RESULTS_PATH = _os.path.join(_THIS_DIR, "ik_results.json")


def load_sampled_poses(path=SAMPLED_POSES_PATH):
    """Reads the JSON written by export_sampled_poses.py (run in the Kit
    Script Editor -- see that file's docstring for why this is a separate
    process/file handoff rather than a direct import)."""
    import json
    with open(path) as f:
        data = json.load(f)
    return [(tuple(entry["position_xyz"]), tuple(entry["quat_xyzw"])) for entry in data]


def save_results(results, path=IK_RESULTS_PATH):
    """results: list of (pos, quat, joint_positions) from run_batch's return
    value. Written back to disk so a later step (e.g. commanding the robot,
    or filtering a capture loop to only IK-reachable poses) can consume it
    without re-running the solve."""
    import json
    out = [
        {"position_xyz": list(pos), "quat_xyzw": list(quat), "joint_positions": list(joints)}
        for pos, quat, joints in results
    ]
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"Wrote {len(out)} solved poses to {path}")


if __name__ == "__main__":
    import os

    if not os.path.exists(SAMPLED_POSES_PATH):
        print(f"No sampled-poses file found at {SAMPLED_POSES_PATH}.")
        print("Run export_sampled_poses.py in the Kit Script Editor first.")
        print("Falling back to a 2-pose smoke test to confirm the service itself works:")
        example_poses = [
            ((0.3, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
            ((0.35, 0.1, 0.25), (0.0, 0.0, 0.0, 1.0)),
        ]
        run_batch(example_poses)
    else:
        sampled_poses = load_sampled_poses()
        print(f"Loaded {len(sampled_poses)} sampled poses from {SAMPLED_POSES_PATH}")
        results = run_batch(sampled_poses)
        if results:
            save_results(results)


