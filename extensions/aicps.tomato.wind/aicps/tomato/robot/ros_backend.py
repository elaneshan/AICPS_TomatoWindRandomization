"""
ros_backend.py -- outside-Kit ManipulationBackend implementation.

Runs as a PLAIN PYTHON SCRIPT in a normal terminal with system ROS2
Humble sourced -- never inside Kit. 

"""

import rclpy

from manipulation_backend import ManipulationBackend
import compute_ik_client
from execute_trajectory_client import TrajectoryExecutorClient
import manipulability_check
import mailbox_client


class ROSBackend(ManipulationBackend):
    def __init__(self):
        rclpy.init()
        self._ik_client = compute_ik_client.ComputeIKClient()
        self._traj_client = TrajectoryExecutorClient()
        self._manip_fk_client = manipulability_check.FKClient()
        self._mailbox = mailbox_client.MailboxClient()


    def solve_ik(self, target):
        success, joints, err_code, seed_label, attempts = self._ik_client.solve_multi_seed(
            target["position_xyz"], target["quat_xyzw"]
        )
        if not success:
            print(f"[ROSBackend] IK failed for target={target.get('target_prim_name')} "
                  f"(error_code={err_code}, {attempts} attempts)")
            return None
        target["seed_used"] = seed_label
        return joints

    def check_manipulability(self, joint_positions):
        J = self._manip_fk_client.numerical_jacobian(joint_positions)
        manip_idx, min_sv, cond = manipulability_check.manipulability_metrics(J)
        return {
            "manipulability_index": manip_idx,
            "min_singular_value": min_sv,
            "condition_number": cond,
        }

    def execute_trajectory(self, joint_positions):
        success, error_code = self._traj_client.send_joint_goal(joint_positions)
        return success, {"error_code": error_code}

    # --- now routed through the mailbox ------------------------------------

    def sample_target(self):
        # mailbox_listener._call_sync returns self.sim.sample_target()'s
        # dict AS-IS (not wrapped) -- matches SimBackend's original
        # return shape exactly, no unwrapping needed here.
        return self._mailbox.sample_target()

    def sample_standoff_target(self):
       return self._mailbox.sample_standoff_target()


    def capture_observation(self, episode_id):
        # Same -- returned as-is, matches episode_capture.capture_episode_frame()'s
        # {"output_dir": ...} shape (or whatever else it eventually returns).
        return self._mailbox.capture_observation(episode_id)

    def move_gripper(self, target_deg):
        # mailbox_listener._call_sync WRAPS this one:
        #   {"success": success, "info": info}
        # -- unwrap here to match ManipulationBackend's
        # (success: bool, info: dict) contract.
        result = self._mailbox.move_gripper(target_deg)
        return result["success"], result["info"]

    def randomize_scene(self):
        # mailbox_listener._call_sync wraps this defensively as
        # {"result": ...} (its real return shape from scene.py was
        # unconfirmed at mailbox-build time -- hand-off v16 SS4).
        # Unwrap the outer dict; whatever's inside is scene.py's own
        # return value, untouched.
        result = self._mailbox.randomize_scene()
        return result.get("result")

    def reset(self):
        grip_success, grip_info = self.move_gripper(-37.24)  # gripper_sync.py's confirmed fully-open value
        if not grip_success:
            print(f"[ROSBackend] WARNING: open-gripper-before-reset failed ({grip_info}) -- "
                  f"retracting anyway, but whatever was grasped may get dragged.")

        success, error_code = self._traj_client.send_joint_goal(compute_ik_client.HOME_SEED)
        if not success:
            print(f"[ROSBackend] WARNING: reset-to-home failed (error_code={error_code}) -- "
                  f"next episode's IK will NOT be starting from a known pose.")
        return success

    # --- lifecycle, not part of the ABC ----------------------------------

    def shutdown(self):
        self._ik_client.destroy_node()
        self._traj_client.destroy_node()
        self._mailbox.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # Smoke test: exercise every method once, in the same order
    # run_episode() would call them -- but individually printed, not
    # yet wired into manipulation_episode.py's actual loop. 
    backend = ROSBackend()
    try:
        scene_result = backend.randomize_scene()
        print(f"randomize_scene -> {scene_result}")

        target = backend.sample_target()
        print(f"sample_target -> {target['target_prim_name']}")

        joints = backend.solve_ik(target)
        print(f"solve_ik -> {joints}")
        if joints is None:
            raise SystemExit("IK failed on sampled target -- try again, this is expected "
                              "to fail sometimes per the ~90-97% solve rate seen in Test C.")

        manip = backend.check_manipulability(joints)
        print(f"check_manipulability -> {manip}")

        success, exec_info = backend.execute_trajectory(joints)
        print(f"execute_trajectory -> success={success} info={exec_info}")
        if not success:
            raise SystemExit("Trajectory execution failed -- stopping before gripper/capture.")

        grip_success, grip_info = backend.move_gripper(target.get("gripper_target_deg", -20.0))
        print(f"move_gripper -> success={grip_success} info={grip_info}")

        observation = backend.capture_observation(episode_id=0)
        print(f"capture_observation -> {observation}")

        reset_success = backend.reset()
        print(f"reset -> success={reset_success}")
    finally:
        backend.shutdown()


