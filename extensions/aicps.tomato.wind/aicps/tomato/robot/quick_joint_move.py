"""
quick_joint_move.py -- moves the arm through 3 hand-picked poses, gripper
untouched, to visually check camera framing at real (non-home) configurations.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ACTION_NAME = "/cr3_group_controller/follow_joint_trajectory"
TIME_TO_REACH_SEC = 3.0

# Swap these for real values from a PLAUSIBLE-classified entry in your
# sampled_poses_seed*.json once you have one you want to inspect specifically.
TEST_POSES = {
    "home":  [0.0, 0.6072, -1.7223, -0.2949, 1.6134, 0.0],
    "pose_a": [-0.33, -1.233, 0.856, 1.185, -0.25, -0.792],   # from your own ik_results_seed42
    "pose_b": [0.522, 1.105, -0.27, -1.831, 0.107, 0.994],    # from your own ik_results_seed42
}


class QuickJointMover(Node):
    def __init__(self):
        super().__init__("quick_joint_mover")
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.get_logger().info(f"Waiting for action server at {ACTION_NAME} ...")
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server {ACTION_NAME} not available after 5s.")
        self.get_logger().info("Action server found.")

    def send_joint_goal(self, joint_positions, time_to_reach_sec=TIME_TO_REACH_SEC):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start = Duration(seconds=time_to_reach_sec).to_msg()
        goal_msg.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=time_to_reach_sec + 5.0)
        if not send_future.done() or not send_future.result().accepted:
            self.get_logger().error("Goal not accepted.")
            return False

        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=time_to_reach_sec + 5.0)
        if not result_future.done():
            self.get_logger().error("Timed out waiting for result.")
            return False

        result = result_future.result().result
        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(f"{'OK' if success else 'FAIL'} (error_code={result.error_code})")
        return success


if __name__ == "__main__":
    rclpy.init()
    try:
        mover = QuickJointMover()
        for name, joints in TEST_POSES.items():
            print(f"\n--- moving to '{name}': {joints} (time_to_reach={TIME_TO_REACH_SEC}s) ---")
            mover.send_joint_goal(joints)
            input(f"At '{name}' -- check RViz/viewport for arm wobble and camera angle, press Enter to continue...")
        mover.destroy_node()
    finally:
        rclpy.shutdown()

