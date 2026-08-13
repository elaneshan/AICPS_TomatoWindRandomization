"""
gripper_sync.py -- manual software replacement for PhysxMimicJointAPI on the
AG-105-145 gripper, working around a confirmed Isaac Sim 6.0.1 PhysX bug.

USAGE:
    import gripper_sync
    gripper_sync.start_gripper_sync()      # call once per Isaac Sim session, after Play
    gripper_sync.set_gripper_target_deg(-20)   # command the gripper anywhere else in your code
    ...
    gripper_sync.stop_gripper_sync()       # call to stop the background sync loop
"""

import omni.kit.app
import omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema
import numpy as np
from isaacsim.core.prims import Articulation

ROBOT_PATH = "/World/cr3/Geometry/world/dummy_link/base_link"
GRIPPER_PHYSICS_PATH = ROBOT_PATH + "/Link1/Link2/Link3/Link4/Link5/Link6/Gripper/Physics"

MASTER_JOINT_NAME = "gripper_finger1_joint"
MASTER_DOF_INDEX = 6

# Gearing ratios mechanically derived from the URDF's mimic multipliers
# (see handoff doc history) -- these are the values PhysxMimicJointAPI was
# configured with before it was removed as part of this workaround.
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

# Drive tuning -- matches the master's own confirmed-working values.
# NOT re-derived/re-tuned for the followers specifically; revisit if any
# follower sags, oscillates, or fights the master under real load.
DRIVE_STIFFNESS = 2000.0
DRIVE_DAMPING = 200.0
DRIVE_MAX_FORCE = 100000.0

_stage = omni.usd.get_context().get_stage()
_gripper_root = _stage.GetPrimAtPath(GRIPPER_PHYSICS_PATH)
_articulation = Articulation(prim_paths_expr=ROBOT_PATH)
_update_sub = None


def _strip_mimic_apis():
    """Remove any PhysxMimicJointAPI instances -- we no longer use the
    built-in mimic feature, see module docstring for why."""
    for prim in Usd.PrimRange(_gripper_root):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            for axis in ["rotX", "rotY", "rotZ"]:
                mimic = PhysxSchema.PhysxMimicJointAPI.Get(prim, axis)
                if mimic:
                    prim.RemoveAPI(PhysxSchema.PhysxMimicJointAPI, axis)


def _ensure_follower_drives():
    """Make sure every follower joint has its own real, working drive --
    the manual sync loop still needs a motor to actually move each joint."""
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
    """Runs every simulation frame while the sync loop is active: reads the
    master's live position, computes each follower's target via its gearing
    ratio, and writes all 7 follower targets in one call."""
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


def start_gripper_sync():
    """Call once per Isaac Sim session, after Play. Safe to call again --
    won't stack a duplicate loop if one is already running."""
    global _update_sub
    if _update_sub is not None:
        print("gripper_sync: already running, not starting a second loop.")
        return

    _strip_mimic_apis()
    _ensure_follower_drives()
    _articulation.initialize()

    stream = omni.kit.app.get_app().get_update_event_stream()
    _update_sub = stream.create_subscription_to_pop(_on_update, name="gripper_mimic_workaround")
    print("gripper_sync: started. Followers will now track the master every frame.")


def stop_gripper_sync():
    """Stops the background per-frame sync loop. Master/followers stay
    wherever they currently are."""
    global _update_sub
    if _update_sub is not None:
        _update_sub.unsubscribe()
        _update_sub = None
        print("gripper_sync: stopped.")
    else:
        print("gripper_sync: was not running.")


def set_gripper_target_deg(target_deg):
    """Command the gripper's master joint to a target angle in degrees.
    Valid range per the URDF: -37.24 (closed) to 0.0 (open).
    Followers will track automatically via the running sync loop."""
    target_rad = np.radians(target_deg)
    _articulation.set_joint_position_targets(
        positions=np.array([target_rad]),
        joint_indices=np.array([MASTER_DOF_INDEX])
    )

