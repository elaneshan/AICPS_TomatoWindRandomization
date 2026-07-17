from dataclasses import dataclass
from .constants import (
    DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE,
    DEFAULT_MIN_ANGLE_X, DEFAULT_MAX_ANGLE_X,
    DEFAULT_MIN_ANGLE_Z, DEFAULT_MAX_ANGLE_Z,
)

_AXIS_DEFAULTS = {
    "x": (DEFAULT_MIN_ANGLE_X, DEFAULT_MAX_ANGLE_X),
    "y": (DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE),
    "z": (DEFAULT_MIN_ANGLE_Z, DEFAULT_MAX_ANGLE_Z),
}


# TODO: we will have to go through and manually check the constraints for each pedicel and leaf and add that value to constants later...
# built this way to keep it modular and we can chain constraints with collision checks
# when building the randomizer

# not used anymore, but dont delete it yet
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






def apply_default_constraints(rig, overwrite=False):
    """Populates axis_limits on every PedicelRigData, per axis, using
    _AXIS_DEFAULTS. Set overwrite=True to replace already-set axes."""
    applied = 0
    for pedicel in rig.pedicels:
        changed = False
        for axis, (default_min, default_max) in _AXIS_DEFAULTS.items():
            current_min, current_max = pedicel.axis_limits[axis]
            if current_min is not None and current_max is not None and not overwrite:
                continue
            pedicel.axis_limits[axis] = (default_min, default_max)
            changed = True
        if changed:
            applied += 1
    return applied


def apply_per_pedicel_constraints(rig, overrides: dict):
    """overrides: {"Pedicel_01": {"x": (min,max), "y": (min,max)}, ...}
    Any axis omitted for a pedicel keeps whatever it already had."""
    applied = []
    for pedicel in rig.pedicels:
        name = pedicel.prim.GetName()
        if name in overrides:
            for axis, limits in overrides[name].items():
                pedicel.axis_limits[axis] = limits
            applied.append(name)
    return applied


def validate_pedicel_angle(pedicel_rig_data, axis: str, angle: float) -> bool:
    min_a, max_a = pedicel_rig_data.axis_limits[axis]
    default_min, default_max = _AXIS_DEFAULTS[axis]
    min_a = min_a if min_a is not None else default_min
    max_a = max_a if max_a is not None else default_max
    return min_a <= angle <= max_a


def try_rotate(controller_tool, pedicel_rig_data, x_deg, y_deg, z_deg):
    if not all([
        validate_pedicel_angle(pedicel_rig_data, "x", x_deg),
        validate_pedicel_angle(pedicel_rig_data, "y", y_deg),
        validate_pedicel_angle(pedicel_rig_data, "z", z_deg),
    ]):
        return False
    controller_tool.rotate(pedicel_rig_data, x_deg, y_deg, z_deg)
    return True

