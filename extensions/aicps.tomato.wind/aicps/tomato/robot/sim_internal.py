"""
sim_internal.py -- Isaac-Sim/Kit-only manipulation internals.
"""
import sys
import asyncio
import os

ROBOT_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/robot"
WIND_PKG_DIR = "/home/aicps/AICPS_TomatoWindRandomization/extensions/aicps.tomato.wind/aicps/tomato/wind"
for p in (ROBOT_PKG_DIR, WIND_PKG_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import omni.usd as usd
import omni.replicator.core as rep

import gripper_sync
import wrist_camera_lookat

import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.collisions as collisions_module
import aicps.tomato.wind.transform as transform_module
import aicps.tomato.wind.scene as scene_module

import manipulation_targets
import episode_capture


ROBOT_BASE_PATH = "/World/cr3/Geometry/world/dummy_link/base_link"

EYE_IN_HAND_CAMERA_PATH = (
    "/World/cr3/Geometry/world/dummy_link/base_link/Link1/Link2/Link3/"
    "Link4/Link5/Link6/Gripper/Geometry/gripper_base_link/EyeInHand_Camera"
)
CAPTURE_RESOLUTION = (1280, 720)

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
    Owns the rig/checker/controller/gripper-sync state inside Kit.

    CONSTRUCTION IS TWO-PHASE, on purpose:
      1. __init__ (sync) -- builds the rig/checker/controller, kicks off
         gripper_sync.start_gripper_sync() (which finishes asynchronously
         a few frames later), and does NOT touch render_product/writer.
      2. await setup_capture() -- must be called AFTER gripper_ready()
         succeeds. Builds the persistent render_product/writer only once
         gripper_sync's own fragile startup window has fully passed.

    The caller (mailbox_listener.py) is responsible for calling both
    phases in order before servicing any requests.
    """

    def __init__(self, capture_output_dir="/home/aicps/manipulation_episodes"):
        self.stage = usd.get_context().get_stage()

        self.rig = rig_module.PlantRig(self.stage, leaf_pairing_overrides=LEAF_PAIRING_OVERRIDES)
        self.rig.build()

        self.checker = collisions_module.CollisionChecker(self.stage, leaf_rig_items=self.rig.leaves)
        self.checker.capture_baselines(self.rig)

        self.controller_tool = transform_module.TransformController(self.stage)

        self.robot_base_prim = self.stage.GetPrimAtPath(ROBOT_BASE_PATH)
        if not self.robot_base_prim.IsValid():
            raise RuntimeError(f"No prim at {ROBOT_BASE_PATH}")

        self.capture_output_dir = capture_output_dir
        os.makedirs(self.capture_output_dir, exist_ok=True)

        gripper_sync.start_gripper_sync()
        wrist_camera_lookat.start_camera_lookat()

        # NOT built here -- see setup_capture(). Left None so any
        # accidental early use of capture_observation() fails loudly
        # (AttributeError on None) rather than silently doing something
        # wrong.
        self._render_product = None
        self._writer = None
        self._next_frame_index = 0
        self._capture_ready = False

    # --- called by the mailbox listener, one request type each --------

    async def gripper_ready(self, timeout_sec=GRIPPER_READY_TIMEOUT_SEC):
        """Blocks (async, polling) until gripper_sync reports ready, or
        raises on timeout. Call once after construction, before
        setup_capture() and before the listener starts servicing
        move_gripper requests."""
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

    async def setup_capture(self):
        self._render_product = rep.create.render_product(EYE_IN_HAND_CAMERA_PATH, CAPTURE_RESOLUTION)
        self._render_product.hydra_texture.set_updates_enabled(False)

        self._writer = rep.writers.get("BasicWriter")
        self._writer.initialize(
            output_dir=self.capture_output_dir, rgb=True, distance_to_camera=True
        )
        self._writer.attach([self._render_product])

        self._capture_ready = True
        print("SimInternal: capture render_product/writer ready.")





    def randomize_scene(self):
        return scene_module.randomize_scene(
            self.stage, self.rig, self.checker, self.controller_tool,
        )

    def sample_target(self):
        target = manipulation_targets.sample_grasp_target(self.rig, self.robot_base_prim)
        if target is None:
            raise RuntimeError("rig has no pedicels or leaves -- did rig.build() run on the right stage?")
        return target

    def sample_standoff_target(self):
       target = manipulation_targets.sample_standoff_target(self.rig, self.robot_base_prim)
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
        return self.move_gripper(-37.24)

    async def capture_observation(self, episode_id):
        if not self._capture_ready:
            raise RuntimeError(
                "SimInternal.setup_capture() has not completed -- capture "
                "requests cannot be serviced yet."
            )

        frame_index = self._next_frame_index
        self._next_frame_index += 1

        result = await episode_capture.capture_episode_frame(
            self._render_product, self._writer, self.capture_output_dir, episode_id, frame_index
        )
        result["frame_index"] = frame_index
        print(f"SimInternal: captured frame_index={frame_index} for "
              f"episode_id={episode_id}, dir={result['episode_dir']}")
        return result




    async def shutdown(self):
        if self._capture_ready:
            await rep.orchestrator.wait_until_complete_async()
            self._writer.detach()
            self._render_product.destroy()
            self._capture_ready = False
        gripper_sync.stop_gripper_sync()
        wrist_camera_lookat.stop_camera_lookat()






