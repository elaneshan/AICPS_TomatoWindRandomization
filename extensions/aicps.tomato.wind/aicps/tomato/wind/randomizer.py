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


def ensure_controller(rig_item, controller_tool):
    if not rig_item.controller_created:
        controller_tool.create_rotation_root(rig_item)

# flutter will be around 1.75 degrees with gausian 
def sample_rotation(rig_item, coupled_pedicel=None, flutter_sigma_fraction=0.52):
    """Samples all three axes. If coupled_pedicel is given, the y axis
    is NOT an independent draw — it's the paired pedicel's current y
    plus a small independent flutter, clamped to this item's own y range."""
    from .constraints import _AXIS_DEFAULTS

    angles = {}
    for axis in ("x", "y", "z"):
        min_a, max_a = rig_item.axis_limits[axis]
        default_min, default_max = _AXIS_DEFAULTS[axis]
        min_a = min_a if min_a is not None else default_min
        max_a = max_a if max_a is not None else default_max

        if axis == "y" and coupled_pedicel is not None:
            base = coupled_pedicel.current_rotation["y"]
            half_range = (max_a - min_a) / 2.0
            flutter = random.gauss(0.0, half_range * flutter_sigma_fraction)
            angles[axis] = max(min_a, min(max_a, base + flutter))
        else:
            angles[axis] = sample_angle(min_a, max_a)
    return angles




def check_against_all(rig, checker, rig_item, debug=False):
    """Checks rig_item against everything relevant to its type. Pedicels
    get pedicel-vs-pedicel + trellis + pedicel-vs-leaf. Leaves get
    leaf-vs-leaf. (Leaf-vs-trellis isn't built yet - flagging, not solving.)"""
    is_pedicel = rig_item in rig.pedicels

    if is_pedicel:
        for other in rig.pedicels:
            if other is rig_item:
                continue
            rejected, info = checker.check_pedicel_pair(rig_item, other, debug=debug)
            if rejected:
                return True, {"against": other.prim.GetName(), **info}

        if checker.environment_prim is not None:
            env_rejected, env_info = checker.check_environment_collision(rig_item, debug=debug)
            if env_rejected:
                return True, {"against": "trellis", **env_info}

    if checker.leaf_items:
        if is_pedicel:
            leaf_rejected, leaf_info = checker.check_leaf_collision(rig_item, debug=debug)
        else:
            leaf_rejected, leaf_info = checker.check_leaf_leaf_collision(rig_item, debug=debug)
        if leaf_rejected:
            return True, {"against": "leaves", **leaf_info}
    
    # Check environment collision for leaves 
    if not is_pedicel and checker.environment_prim is not None:
        env_rejected, env_info = checker.check_leaf_environment_collision(rig_item, debug=debug)
        if env_rejected:
            return True, {"against": "trellis", **env_info}
    # Check against own pedicel (coupling)
    if not is_pedicel and rig_item.paired_pedicel_name:
        pedicel_lookup = next((p for p in rig.pedicels if p.prim.GetName() == rig_item.paired_pedicel_name), None)
        if pedicel_lookup is not None:
            own_rejected, own_info = checker.check_leaf_against_own_pedicel(rig_item, pedicel_lookup, debug=debug)
            if own_rejected:
                return True, {"against": rig_item.paired_pedicel_name, **own_info}

    

    return False, None





def randomize_item(rig, checker, controller_tool, rig_item, max_attempts=20, debug=False, coupled_pedicel=None):
    from .constraints import _AXIS_DEFAULTS

    ensure_controller(rig_item, controller_tool)

    for attempt in range(max_attempts):
        angles = sample_rotation(rig_item, coupled_pedicel=coupled_pedicel)

        if not all(validate_pedicel_angle(rig_item, axis, a) for axis, a in angles.items()):
            continue

        controller_tool.rotate(rig_item, angles["x"], angles["y"], angles["z"])
        rejected, info = check_against_all(rig, checker, rig_item, debug=debug)

        if not rejected:
            if debug:
                print(f"  ACCEPTED {rig_item.prim.GetName()} @ "
                      f"x={angles['x']:.2f} y={angles['y']:.2f} z={angles['z']:.2f} deg "
                      f"(attempt {attempt + 1})")
            return True

        if debug:
            print(f"  reject {rig_item.prim.GetName()} @ "
                  f"x={angles['x']:.2f} y={angles['y']:.2f} z={angles['z']:.2f} deg "
                  f"(attempt {attempt + 1}): {info}")

    # Fallback. Coupled leaves fall back to their pedicel's CURRENT y with
    # zero flutter, not global (0,0,0) rest
    if coupled_pedicel is not None:
        min_y, max_y = rig_item.axis_limits["y"]
        default_min, default_max = _AXIS_DEFAULTS["y"]
        min_y = min_y if min_y is not None else default_min
        max_y = max_y if max_y is not None else default_max
        fallback_y = max(min_y, min(max_y, coupled_pedicel.current_rotation["y"]))
        controller_tool.rotate(rig_item, 0.0, fallback_y, 0.0)
    else:
        controller_tool.rotate(rig_item, 0.0, 0.0, 0.0)

    rejected_at_rest, info = check_against_all(rig, checker, rig_item, debug=debug)
    if not rejected_at_rest:
        if debug:
            print(f"  FAILED to find valid pose for {rig_item.prim.GetName()} "
                  f"after {max_attempts} attempts - reset to fallback (verified safe)")
        return False

    print(f"   {rig_item.prim.GetName()}: NO safe pose found, including fallback, "
          f"given current neighbor positions. {info}")
    return False






def randomize_pedicel(rig, checker, controller_tool, pedicel, max_attempts=20, debug=False):
    return randomize_item(rig, checker, controller_tool, pedicel, max_attempts=max_attempts, debug=debug)



def randomize_all(rig, checker, controller_tool, max_attempts=20, seed=None, debug=False):
    if seed is not None:
        random.seed(seed)

    apply_default_constraints(rig)

    leaves_by_pedicel = {}
    unpaired_leaves = []
    for leaf in rig.leaves:
        if leaf.paired_pedicel_name:
            leaves_by_pedicel.setdefault(leaf.paired_pedicel_name, []).append(leaf)
        else:
            unpaired_leaves.append(leaf)

    pedicel_order = list(rig.pedicels)
    random.shuffle(pedicel_order)

    results = {}
    for pedicel in pedicel_order:
        accepted = randomize_item(rig, checker, controller_tool, pedicel, max_attempts=max_attempts, debug=debug)
        results[pedicel.prim.GetName()] = {
            "accepted": accepted,
            "final_rotation": dict(pedicel.current_rotation),
        }

        paired_leaves = leaves_by_pedicel.get(pedicel.prim.GetName(), [])
        random.shuffle(paired_leaves)
        for leaf in paired_leaves:
            accepted = randomize_item(
                rig, checker, controller_tool, leaf,
                max_attempts=max_attempts, debug=debug, coupled_pedicel=pedicel,
            )
            results[leaf.prim.GetName()] = {
                "accepted": accepted,
                "final_rotation": dict(leaf.current_rotation),
            }

    random.shuffle(unpaired_leaves)
    for leaf in unpaired_leaves:
        accepted = randomize_item(rig, checker, controller_tool, leaf, max_attempts=max_attempts, debug=debug)
        results[leaf.prim.GetName()] = {
            "accepted": accepted,
            "final_rotation": dict(leaf.current_rotation),
        }

    return results


