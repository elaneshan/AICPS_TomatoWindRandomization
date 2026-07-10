from dataclasses import dataclass

from .constants import DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE

# TODO: we will have to go through and manually check the constraints for each pedicel and leaf and add that value to constants later...
# built this way to keep it modular and we can chain constraints with collision checks
# when building the randomizer


@dataclass
class RotationConstraint:
    """
    Rotation limits in degrees for a single pedicel (or later, a leaf).
    Placeholder values - these have no biomechanical justification yet,
    just a starting range to validate the accept/reject pipeline itself.
    """
    min_angle: float = DEFAULT_MIN_ANGLE
    max_angle: float = DEFAULT_MAX_ANGLE

    def is_valid(self, angle: float) -> bool:
        return self.min_angle <= angle <= self.max_angle

def apply_default_constraints(rig, min_angle=DEFAULT_MIN_ANGLE, max_angle=DEFAULT_MAX_ANGLE, overwrite=False):
    """
    Populates min_angle/max_angle on every PedicelRigData in the rig.
    Set overwrite=True to replace values that are already set.
    """
    applied = 0
    for pedicel in rig.pedicels:
        if pedicel.min_angle is not None and pedicel.max_angle is not None and not overwrite:
            continue
        pedicel.min_angle = min_angle
        pedicel.max_angle = max_angle
        applied += 1
    return applied

def apply_per_pedicel_constraints(rig, overrides: dict):
    """
    overrides: {"Pedicel_01": (min, max), "Pedicel_08": (-3, 4), ...}
    Data-driven per-pedicel table (matches the PLANT_CONFIG pattern from
    the design doc) instead of hardcoding any pedicel-specific logic
    directly into the constraint class.
    """
    applied = []
    for pedicel in rig.pedicels:
        name = pedicel.prim.GetName()
        if name in overrides:
            pedicel.min_angle, pedicel.max_angle = overrides[name]
            applied.append(name)
    return applied

def validate_pedicel_angle(pedicel_rig_data, angle: float) -> bool:
    """
    True if angle is within this pedicel's constraint. Falls back to
    global defaults if no constraint has been assigned yet - constraints
    should be opt-out (via apply_default_constraints), not silently
    skipped by omission.
    """
    min_a = pedicel_rig_data.min_angle if pedicel_rig_data.min_angle is not None else DEFAULT_MIN_ANGLE
    max_a = pedicel_rig_data.max_angle if pedicel_rig_data.max_angle is not None else DEFAULT_MAX_ANGLE
    return min_a <= angle <= max_a

def try_rotate(controller_tool, pedicel_rig_data, angle):
    """
    Attempts a constrained rotation: validates first, only rotates if
    accepted. Returns True/False rather than clamping - this is the
    reject-and-resample shape Phase 5's randomizer will chain constraint
    checks and collision checks together with, not a fallback clamp.
    """
    if not validate_pedicel_angle(pedicel_rig_data, angle):
        return False
    controller_tool.rotate(pedicel_rig_data, angle)
    return True

