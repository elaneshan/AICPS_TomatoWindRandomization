"""
sim_backend.py -- SimBackend implementation. Construction mirrors
test_pipeline_demo.py exactly (rig -> checker -> controller_tool),
since that's the proven-working setup sequence for a fresh stage.

V1 SCOPE, DELIBERATE: each episode is reach -> grasp -> done. No
"attempt to move the branch and check" logic yet -- that's a real,
separate next step once basic reach-and-grasp is validated.
"""
import sys
import os
import asyncio

ROBOT_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/robot"
WIND_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/wind"
for p in (ROBOT_PKG_DIR, WIND_PKG_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import omni.usd as usd

from manipulation_backend import ManipulationBackend

# i have a feeling that these imports will not work
import gripper_sync
import compute_ik_client
from execute_trajectory_client import TrajectoryExecutorClient
import manipulability_check

import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.collisions as collisions_module
import aicps.tomato.wind.transform as transform_module

import manipulation_targets
import episode_capture


ROBOT_BASE_PATH = "/World/cr3/Geometry/world/dummy_link/base_link"

# From test_pipeline_demo.py -- verified against the viewport per that
# file's own comment. Lives here (not hardcoded per-call) since it's a
# property of THIS cluster asset, same lifetime as the rig itself.
LEAF_PAIRING_OVERRIDES = {
    "foliage_leaf_01": "Pedicel_01",
    "foliage_leaf_07": "Pedicel_01",
    "foliage_leaf_05": "Pedicel_02",
    "foliage_leaf_02": "Pedicel_04",
    "foliage_leaf_06": "Pedicel_08",
    "foliage_leaf_03": "Pedicel_06",
    "foliage_leaf_04": "Pedicel_05",
}

MANIPULABILITY_MIN_SV_THRESHOLD = 0.05  # see manipulation_episode.py note


class SimBackend(ManipulationBackend):
    def __init__(self, capture_output_dir="/home/aicps/manipulation_episodes"):
        self.stage = usd.get_context().get_stage()

        # Mirrors test_pipeline_demo.py's setup exactly.
        self.rig = rig_module.PlantRig(self.stage, leaf_pairing_overrides=LEAF_PAIRING_OVERRIDES)
        self.rig.build()

        self.checker = collisions_module.CollisionChecker(self.stage, leaf_rig_items=self.rig.leaves)
        self.checker.capture_baselines(self.rig)

        self.controller_tool = transform_module.TransformController(self.stage)

        self.robot_base_prim = self.stage.GetPrimAtPath(ROBOT_BASE_PATH)
        if not self.robot_base_prim.IsValid():
            raise RuntimeError(f"No prim at {ROBOT_BASE_PATH}")

        self.capture_output_dir = capture_output_dir

        # Created lazily, not here -- see prior note on avoiding
        # constructing rclpy/Articulation objects before Play/at import time.
        self._ik_client = None
        self._traj_client = None
        self._manip_fk_client = None

    # --- ManipulationBackend interface ----------------------------------

    def randomize_scene(self):
        import scene as scene_module
        return scene_module.randomize_scene(
            self.stage, self.rig, self.checker, self.controller_tool,
        )

    def sample_target(self):
        target = manipulation_targets.sample_grasp_target(self.rig, self.robot_base_prim)
        if target is None:
            raise RuntimeError("rig has no pedicels or leaves -- did rig.build() run on the right stage?")
        return target

    def solve_ik(self, target):
        if self._ik_client is None:
            self._ik_client = compute_ik_client.ComputeIKClient()
        success, joints, err_code, seed_label, attempts = self._ik_client.solve_multi_seed(
            target["position_xyz"], target["quat_xyzw"]
        )
        if not success:
            print(f"[SimBackend] IK failed for target={target['target_prim_name']} "
                  f"(error_code={err_code}, {attempts} attempts)")
            return None
        target["seed_used"] = seed_label
        return joints

    def check_manipulability(self, joint_positions):
        if self._manip_fk_client is None:
            self._manip_fk_client = manipulability_check.FKClient()
        J = self._manip_fk_client.numerical_jacobian(joint_positions)
        manip_idx, min_sv, cond = manipulability_check.manipulability_metrics(J)
        return {
            "manipulability_index": manip_idx,
            "min_singular_value": min_sv,
            "condition_number": cond,
        }

    def execute_trajectory(self, joint_positions):
        if self._traj_client is None:
            self._traj_client = TrajectoryExecutorClient()
        success, error_code = self._traj_client.send_joint_goal(joint_positions)
        return success, {"error_code": error_code}

    def move_gripper(self, target_deg):
        gripper_sync.set_gripper_target_deg(target_deg)
        return True, {}

    def capture_observation(self, episode_id):
        loop = asyncio.get_event_loop()
        result = loop.run_until_complete(
            episode_capture.capture_episode_frame(self.capture_output_dir, episode_id)
        )
        return result

    def reset(self):
        """
        Open gripper FIRST, then retract to HOME_SEED -- releasing before
        retracting avoids dragging/snagging whatever was just grasped as
        the arm moves away. Keeps every episode starting from the same
        known configuration, which is what let the 'zero' IK seed solve
        100% of today's successes -- letting the arm end episodes in
        arbitrary poses would make that no longer reliably true.
        """
        gripper_sync.open_gripper()

        if self._traj_client is None:
            self._traj_client = TrajectoryExecutorClient()
        success, error_code = self._traj_client.send_joint_goal(compute_ik_client.HOME_SEED)
        if not success:
            print(f"[SimBackend] WARNING: reset-to-home failed (error_code={error_code}) -- "
                  f"next episode's IK will NOT be starting from a known pose.")
        return success


