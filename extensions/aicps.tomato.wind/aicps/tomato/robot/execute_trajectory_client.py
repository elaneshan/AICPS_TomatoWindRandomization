"""
execute_trajectory_client.py -- sends solved joint configurations from
ik_results_seed*.json to the robot via MoveIt's FollowJointTrajectory
action, executed through cr3_group_controller.

IMPORTANT SCOPING NOTE: this action talks directly to the controller,
bypassing MoveIt's planner. It gives smoother, interpolated motion than
a raw /isaac_joint_command publish, but it is NOT collision-aware --
that requires going through MoveGroupInterface's plan()/plan_and_execute()
with a populated planning scene instead. Do not treat "this script works"
as "collision avoidance is done."

NOT YET CONFIRMED: the action name below is inferred from the standard
MoveIt ros2_control naming convention (<controller_name>/follow_joint_trajectory),
not confirmed against the actual moveit_controllers.yaml. Confirm this
before trusting a hang/timeout as meaningful -- an action client that
can't find its server will just sit there, not raise a clear error.

Run with dry_run=True first (default in __main__) to confirm the action
server is reachable before ever commanding real motion.
"""
import glob
import json
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Confirmed against moveit_controllers.yaml: cr3_group_controller's
# action_ns is "follow_joint_trajectory" -> full action name below.
ACTION_NAME = "/cr3_group_controller/follow_joint_trajectory"

# Generous first-guess time budget per goal, not tuned. Same
# "start conservative, tune from observed behavior" pattern as v7's
# drive-stiffness placeholders -- adjust once you've watched it run.
TIME_TO_REACH_SEC = 3.0

# Extra wait after the action reports SUCCEEDED, before treating the pose
# as "arrived" for capture purposes -- gives physics a moment to settle.
POST_ARRIVAL_SETTLE_SEC = 0.5


class TrajectoryExecutorClient(Node):
    def __init__(self):
        super().__init__("trajectory_executor_client")
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.get_logger().info(f"Waiting for action server at {ACTION_NAME} ...")
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f"Action server {ACTION_NAME} not available after 5s -- "
                f"is moveit_demo.launch.py running, and is this action "
                f"name actually correct? Check moveit_controllers.yaml "
                f"before assuming the launch file is the problem."
            )
        self.get_logger().info("Action server found.")

    def send_joint_goal(self, joint_positions, time_to_reach_sec=TIME_TO_REACH_SEC):
        """Sends a single-waypoint trajectory goal, blocks until result.
        Returns (success: bool, error_code: int | None)."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start = Duration(seconds=time_to_reach_sec).to_msg()
        goal_msg.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=time_to_reach_sec + 5.0)
        if not send_future.done():
            self.get_logger().error("Goal send timed out (no ack from action server).")
            return False, None

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            return False, None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=time_to_reach_sec + 5.0)
        if not result_future.done():
            self.get_logger().error("Goal timed out waiting for result.")
            return False, None

        result = result_future.result().result
        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not success:
            self.get_logger().error(
                f"Goal finished with error_code={result.error_code}: {result.error_string}"
            )
        return success, result.error_code


def load_ik_results(path):
    with open(path) as f:
        return json.load(f)


def run_all(paths, dry_run=True):
    client = TrajectoryExecutorClient()  # raises immediately if server not found

    if dry_run:
        print("Dry run only -- action server confirmed reachable, no goals sent.")
        client.destroy_node()
        return

    n_ok = n_fail = 0
    for path in paths:
        entries = load_ik_results(path)
        tag = os.path.basename(path)
        print(f"\n=== {tag} ({len(entries)} poses) ===")
        for i, entry in enumerate(entries):
            joints = entry["joint_positions"]
            print(f"[{tag} {i:03d}] sending joint goal: "
                  f"{tuple(round(j, 3) for j in joints)} ...", end=" ")
            success, err = client.send_joint_goal(joints)
            if success:
                n_ok += 1
                print("OK")
                time.sleep(POST_ARRIVAL_SETTLE_SEC)
            else:
                n_fail += 1
                print(f"FAIL (error_code={err})")

    print(f"\n--- summary ---")
    print(f"{n_ok}/{n_ok + n_fail} goals executed successfully")
    client.destroy_node()


if __name__ == "__main__":
    _THIS_DIR = os.path.dirname(os.path.abspath(__file__))
    ik_paths = sorted(glob.glob(os.path.join(_THIS_DIR, "ik_results_seed*.json")))
    single_path = os.path.join(_THIS_DIR, "ik_results.json")
    if not ik_paths and os.path.exists(single_path):
        ik_paths = [single_path]
    if not ik_paths:
        raise RuntimeError("No ik_results*.json found -- run compute_ik_client.py first.")

    rclpy.init()
    try:
        run_all(ik_paths, dry_run=False)
    finally:
        rclpy.shutdown()

