"""
sim_internal.py -- Isaac-Sim/Kit-only manipulation internals.

This is NOT a ManipulationBackend implementation. It's the set of
methods the (not-yet-built) mailbox listener will call inside Kit to
service requests from the outside process. Split out of the old
sim_backend.py per hand-off v16 SS5's mapping table:

  "sim-only parts stay as-is, running inside Kit as the mailbox
   listener's dependencies" -- randomize_scene, the USD/rig
   construction, sample_target, move_gripper (via gripper_sync),
   capture_observation.

CRITICAL: do not import rclpy or any ROS2 message
package (moveit_msgs, sensor_msgs, etc). 
"""
import sys
import asyncio

ROBOT_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/robot"
WIND_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/wind"
for p in (ROBOT_PKG_DIR, WIND_PKG_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import omni.usd as usd

# Kit-only, zero rclpy/ROS2 dependency -- confirmed via code inspection,
# hand-off v16 SS3.
import gripper_sync

import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.collisions as collisions_module
import aicps.tomato.wind.transform as transform_module
import aicps.tomato.wind.scene as scene_module

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

GRIPPER_READY_TIMEOUT_SEC = 10.0


class SimInternal:
    """
    Owns the rig/checker/controller/gripper-sync state inside Kit. One
    instance per running session -- the (future) mailbox listener
    constructs this once and calls methods on it per request, rather
    than rebuilding any of this per-request (which would re-trigger
    the same module-level-caching class of bug flagged in v13 SS1.4 --
    the fix there was "build at use-time, not import-time", not
    "build fresh every single call").
    """

    def __init__(self, capture_output_dir="/home/aicps/manipulation_episodes"):
        self.stage = usd.get_context().get_stage()

        # Mirrors test_pipeline_demo.py's setup sequence exactly.
        self.rig = rig_module.PlantRig(self.stage, leaf_pairing_overrides=LEAF_PAIRING_OVERRIDES)
        self.rig.build()

        self.checker = collisions_module.CollisionChecker(self.stage, leaf_rig_items=self.rig.leaves)
        self.checker.capture_baselines(self.rig)

        self.controller_tool = transform_module.TransformController(self.stage)

        self.robot_base_prim = self.stage.GetPrimAtPath(ROBOT_BASE_PATH)
        if not self.robot_base_prim.IsValid():
            raise RuntimeError(f"No prim at {ROBOT_BASE_PATH}")

        self.capture_output_dir = capture_output_dir

        # --- FIX (this session) -------------------------------------
        # The old sim_backend.py never called start_gripper_sync() at
        # all. move_gripper() -> gripper_sync.set_gripper_target_deg()
        # hard-requires the sync loop to already be running and ready
        # (it raises RuntimeError otherwise, by design -- see
        # gripper_sync.py's own docstring) -- so the very first
        # move_gripper() call in ANY episode would have raised, every
        # single time, before this fix.
        #
        # start_gripper_sync() itself finishes ASYNCHRONOUSLY (a few
        # real frames after Play, per its own _deferred_start()) -- so
        # kicking it off here is necessary but not sufficient. Callers
        # must `await gripper_ready()` once, after construction, before
        # trusting the first move_gripper() call in a fresh session.
        gripper_sync.start_gripper_sync()

    # --- called by the mailbox listener, one request type each --------

    async def gripper_ready(self, timeout_sec=GRIPPER_READY_TIMEOUT_SEC):
        """Blocks (async, polling) until gripper_sync reports ready, or
        raises on timeout. Call once after construction, before the
        listener starts servicing move_gripper requests."""
        elapsed = 0.0
        poll_interval = 0.1
        while not gripper_sync.is_ready():
            await asyncio.sleep(poll_interval)
            elapsed += poll_interval
            if elapsed >= timeout_sec:
                raise RuntimeError(
                    f"gripper_sync did not become ready within {timeout_sec}s -- "
                    f"check the timeline is actually playing and the robot is "
                    f"referenced into this stage."
                )

    def randomize_scene(self):
        return scene_module.randomize_scene(
            self.stage, self.rig, self.checker, self.controller_tool,
        )

    def sample_target(self):
        target = manipulation_targets.sample_grasp_target(self.rig, self.robot_base_prim)
        if target is None:
            raise RuntimeError("rig has no pedicels or leaves -- did rig.build() run on the right stage?")
        return target

    def move_gripper(self, target_deg):
        if not gripper_sync.is_ready():
            raise RuntimeError(
                "gripper_sync not ready -- await gripper_ready() before "
                "servicing the first move_gripper() request in a session."
            )
        gripper_sync.set_gripper_target_deg(target_deg)
        return True, {}

    def open_gripper(self):
        """Used by the reset flow -- open BEFORE retracting, so a grasped
        object isn't dragged as the arm moves away (same reasoning as the
        original SimBackend.reset())."""
        return self.move_gripper(-37.24)

    async def capture_observation(self, episode_id):
        # This object lives inside Kit, so a plain `await` here runs on
        # Kit's OWN already-running event loop -- correct, and avoids the
        # untested asyncio.get_event_loop().run_until_complete(...) pattern
        # the old SimBackend used (flagged as a real unknown in v14 SS3.5;
        # calling run_until_complete() on an already-running loop typically
        # raises "This event loop is already running"). The mailbox
        # listener will call this as `await sim.capture_observation(...)`
        # from its own async dispatch.
        return await episode_capture.capture_episode_frame(self.capture_output_dir, episode_id)

