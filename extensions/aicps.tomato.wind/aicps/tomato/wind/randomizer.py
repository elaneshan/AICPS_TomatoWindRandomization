import random

from .constraints import apply_default_constraints, validate_pedicel_angle


def sample_angle(min_angle, max_angle, sigma_fraction=0.4):
    """Unchanged - still samples one axis's angle from a gaussian
    centered at the midpoint, clipped to range. Called once per axis now."""
    center = (min_angle + max_angle) / 2.0
    half_range = (max_angle - min_angle) / 2.0
    sigma = half_range * sigma_fraction
    angle = random.gauss(center, sigma)
    return max(min_angle, min(max_angle, angle))


def sample_rotation(pedicel_rig_data):
    """Samples all three axes together, every call - no staged/partial
    resampling. Each axis's range comes from its own axis_limits entry,
    falling back to that axis's default if unset (same convention
    validate_pedicel_angle already uses)."""
    from .constraints import _AXIS_DEFAULTS

    angles = {}
    for axis in ("x", "y", "z"):
        min_a, max_a = pedicel_rig_data.axis_limits[axis]
        default_min, default_max = _AXIS_DEFAULTS[axis]
        min_a = min_a if min_a is not None else default_min
        max_a = max_a if max_a is not None else default_max
        angles[axis] = sample_angle(min_a, max_a)
    return angles


def ensure_controller(pedicel_rig_data, controller_tool):
    if not pedicel_rig_data.controller_created:
        controller_tool.create_rotation_root(pedicel_rig_data)


def check_against_all(rig, checker, pedicel, debug=False):
    """Unchanged - operates purely on world-space geometry after rotation,
    doesn't need to know how many axes produced the current pose."""
    for other in rig.pedicels:
        if other is pedicel:
            continue
        rejected, info = checker.check_pedicel_pair(pedicel, other, debug=debug)
        if rejected:
            return True, {"against": other.prim.GetName(), **info}

    if checker.environment_prim is not None:
        env_rejected, env_info = checker.check_environment_collision(pedicel, debug=debug)
        if env_rejected:
            return True, {"against": "trellis", **env_info}

    if checker.leaf_prims:
        leaf_rejected, leaf_info = checker.check_leaf_collision(pedicel, debug=debug)
        if leaf_rejected:
            return True, {"against": "leaves", **leaf_info}

    return False, None


def randomize_pedicel(rig, checker, controller_tool, pedicel, max_attempts=20, debug=False):
    ensure_controller(pedicel, controller_tool)

    for attempt in range(max_attempts):
        angles = sample_rotation(pedicel)

        if not all(validate_pedicel_angle(pedicel, axis, a) for axis, a in angles.items()):
            continue

        controller_tool.rotate(pedicel, angles["x"], angles["y"], angles["z"])
        rejected, info = check_against_all(rig, checker, pedicel, debug=debug)

        if not rejected:
            if debug:
                print(f"  ACCEPTED {pedicel.prim.GetName()} @ "
                      f"x={angles['x']:.2f} y={angles['y']:.2f} z={angles['z']:.2f} deg "
                      f"(attempt {attempt + 1})")
            return True

        if debug:
            print(f"  reject {pedicel.prim.GetName()} @ "
                  f"x={angles['x']:.2f} y={angles['y']:.2f} z={angles['z']:.2f} deg "
                  f"(attempt {attempt + 1}): {info}")

    # All sampled attempts rejected - verify (0,0,0) against CURRENT
    # neighbor state, same fix as the single-axis version (never assume
    # rest is safe just because it's rest).
    controller_tool.rotate(pedicel, 0.0, 0.0, 0.0)
    rejected_at_rest, info = check_against_all(rig, checker, pedicel, debug=debug)
    if not rejected_at_rest:
        if debug:
            print(f"  FAILED to find valid pose for {pedicel.prim.GetName()} "
                  f"after {max_attempts} attempts - reset to (0,0,0) (verified safe)")
        return False

    print(f"   {pedicel.prim.GetName()}: NO safe pose found, including (0,0,0), "
          f"given current neighbor positions. {info}")
    return False


def randomize_all(rig, checker, controller_tool, max_attempts=20, seed=None, debug=False):
    if seed is not None:
        random.seed(seed)

    apply_default_constraints(rig)
    processing_order = list(rig.pedicels)
    random.shuffle(processing_order)
    results = {}
    for pedicel in processing_order:
        accepted = randomize_pedicel(rig, checker, controller_tool, pedicel, max_attempts=max_attempts, debug=debug)
        results[pedicel.prim.GetName()] = {
            "accepted": accepted,
            "final_rotation": dict(pedicel.current_rotation),  # was final_angle
        }

    return results


