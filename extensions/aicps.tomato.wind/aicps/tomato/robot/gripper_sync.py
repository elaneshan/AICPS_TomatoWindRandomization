"""
gripper_sync.py -- manual software replacement for PhysxMimicJointAPI on the
AG-105-145 gripper, working around a confirmed Isaac Sim 6.0.1 PhysX bug.

USAGE:
    import aicps.tomato.robot.gripper_sync as gripper_sync
    gripper_sync.start_gripper_sync()          # call once per session, works from any timeline state
    # wait for the "ready" print, then:
    gripper_sync.set_gripper_target_deg(-20)   # or gripper_sync.open_gripper() / close_gripper()
    ...
    gripper_sync.stop_gripper_sync()
"""

import asyncio
import omni.kit.app
import omni.usd
import omni.timeline
from pxr import Usd, UsdPhysics, PhysxSchema
import numpy as np
from isaacsim.core.prims import Articulation

ROBOT_PATH = "/World/cr3/Geometry/world/dummy_link/base_link"
GRIPPER_PHYSICS_PATH = ROBOT_PATH + "/Link1/Link2/Link3/Link4/Link5/Link6/Gripper/Physics"

MASTER_JOINT_NAME = "gripper_finger1_joint"
MASTER_DOF_INDEX = 6

# Gearing ratios mechanically derived from the URDF's mimic multipliers.
FOLLOWER_GEARING = {
    "gripper_finger2_joint": 1.0,
    "gripper_finger1_finger_joint": 0.5,
    "gripper_finger2_finger_joint": 0.5,
    "gripper_finger1_inner_knuckle_joint": 1.49,
    "gripper_finger2_inner_knuckle_joint": 1.49,
    "gripper_finger1_finger_tip_joint": 1.49,
    "gripper_finger2_finger_tip_joint": 1.49,
}

# Order must match each name's real articulation DOF index below.
# Confirmed (2026-08-13, this session) via get_dof_names() that this ordering
# is currently correct -- re-verify if the robot is ever re-imported/re-referenced.
FOLLOWER_NAMES_IN_DOF_ORDER = [
    "gripper_finger2_joint",
    "gripper_finger1_inner_knuckle_joint",
    "gripper_finger2_inner_knuckle_joint",
    "gripper_finger1_finger_joint",
    "gripper_finger2_finger_joint",
    "gripper_finger1_finger_tip_joint",
    "gripper_finger2_finger_tip_joint",
]
FOLLOWER_DOF_INDICES = [7, 8, 9, 10, 11, 12, 13]

# Drive tuning -- currently copied from the master's own confirmed-working
# values. NOT yet independently validated for followers under real load.
# See the drive-sweep test (separate script) before trusting these long-term.
DRIVE_STIFFNESS = 2000.0
DRIVE_DAMPING = 200.0
DRIVE_MAX_FORCE = 100000.0

FRAMES_TO_WAIT_AFTER_PLAY = 5  # matches the proven-working isolated test

_stage = None
_gripper_root = None
_articulation = None
_update_sub = None
_ready = False


def _strip_mimic_apis():
    for prim in Usd.PrimRange(_gripper_root):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            for axis in ["rotX", "rotY", "rotZ"]:
                mimic = PhysxSchema.PhysxMimicJointAPI.Get(prim, axis)
                if mimic:
                    prim.RemoveAPI(PhysxSchema.PhysxMimicJointAPI, axis)


def _ensure_follower_drives():
    for prim in Usd.PrimRange(_gripper_root):
        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() != MASTER_JOINT_NAME:
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
            drive.CreateDampingAttr().Set(DRIVE_DAMPING)
            drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)
            drive.CreateTypeAttr().Set("force")


def _on_update(e):
    """Runs every simulation frame while the sync loop is active."""
    if _articulation is None:
        return
    master_pos_rad = _articulation.get_joint_positions(
        joint_indices=np.array([MASTER_DOF_INDEX])
    ).reshape(-1)[0]

    follower_targets = np.array([
        master_pos_rad * FOLLOWER_GEARING[name]
        for name in FOLLOWER_NAMES_IN_DOF_ORDER
    ])

    _articulation.set_joint_position_targets(
        positions=follower_targets,
        joint_indices=np.array(FOLLOWER_DOF_INDICES)
    )


async def _deferred_start():
    """Waits several real frames after Play before touching the Articulation
    object -- matches the exact sequence proven to work in isolated testing.
    Building/initializing the Articulation too soon after Play was observed
    to invalidate the physics tensor simulationView."""
    global _articulation, _update_sub, _ready

    for _ in range(FRAMES_TO_WAIT_AFTER_PLAY):
        await omni.kit.app.get_app().next_update_async()

    _articulation = Articulation(prim_paths_expr=ROBOT_PATH)
    _articulation.initialize()

    stream = omni.kit.app.get_app().get_update_event_stream()
    _update_sub = stream.create_subscription_to_pop(_on_update, name="gripper_mimic_workaround")
    _ready = True
    print("gripper_sync: ready. Followers will now track the master every frame.")


def start_gripper_sync():
    """Call once per session, from any timeline state (playing or stopped).
    Safe to call again -- stops any existing loop first and rebuilds fresh.
    Setup finishes asynchronously; watch for the 'ready' print before
    calling set_gripper_target_deg()."""
    global _stage, _gripper_root, _update_sub, _ready

    if _update_sub is not None:
        stop_gripper_sync()

    _ready = False

    _stage = omni.usd.get_context().get_stage()
    _gripper_root = _stage.GetPrimAtPath(GRIPPER_PHYSICS_PATH)
    if not _gripper_root.IsValid():
        raise RuntimeError(
            f"gripper_sync: no valid prim at {GRIPPER_PHYSICS_PATH} on the "
            f"current stage -- check the robot is referenced into this scene "
            f"and the path hasn't changed."
        )

    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()
    if was_playing:
        timeline.stop()

    _strip_mimic_apis()
    _ensure_follower_drives()

    timeline.play()

    print("gripper_sync: schema fixed, timeline playing, waiting for physics to settle...")
    asyncio.ensure_future(_deferred_start())


def stop_gripper_sync():
    global _update_sub, _ready
    if _update_sub is not None:
        _update_sub.unsubscribe()
        _update_sub = None
        _ready = False
        print("gripper_sync: stopped.")
    else:
        print("gripper_sync: was not running.")


def is_ready():
    return _ready


def set_gripper_target_deg(target_deg):
    """Command the gripper's master joint to a target angle in degrees.

    CONFIRMED BY DIRECT VIEWPORT OBSERVATION (not the URDF's lower/upper
    limit names, which are NOT semantically labeled open/closed):
        0.0 deg    = fully CLOSED
        -37.24 deg = fully OPEN

    Followers track automatically via the running sync loop. Requires
    start_gripper_sync() to have finished (check is_ready() or wait for the
    'ready' print) -- calling this before then will raise, not silently no-op.
    """
    if not _ready or _articulation is None:
        raise RuntimeError(
            "gripper_sync: not ready yet -- call start_gripper_sync() and "
            "wait for the 'ready' print (a few frames) before commanding a target."
        )
    target_rad = np.radians(target_deg)
    _articulation.set_joint_position_targets(
        positions=np.array([target_rad]),
        joint_indices=np.array([MASTER_DOF_INDEX])
    )


def open_gripper():
    """Convenience wrapper: fully open (-37.24 deg, confirmed by viewport)."""
    set_gripper_target_deg(-37.24)


def close_gripper():
    """Convenience wrapper: fully closed (0.0 deg, confirmed by viewport)."""
    set_gripper_target_deg(0.0)

