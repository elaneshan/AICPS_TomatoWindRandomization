import random

from .constraints import apply_default_constraints, validate_pedicel_angle


def sample_angle(min_angle, max_angle, sigma_fraction=0.4):
    """
    Samples an angle from a gaussian centered at the midpoint of
    [min_angle, max_angle], clipped to stay within range.

    sigma_fraction controls how "wide" the natural spread feels - 0.4
    means the standard deviation is 40% of the half-range, so most
    samples land well inside the bounds with only occasional draws near
    the extremes (which then get clipped, slightly stacking probability
    right at the edges - fine for this use case, not a concern unless
    you need a perfectly smooth distribution).
    """
    center = (min_angle + max_angle) / 2.0
    half_range = (max_angle - min_angle) / 2.0
    sigma = half_range * sigma_fraction

    angle = random.gauss(center, sigma)
    return max(min_angle, min(max_angle, angle))



def ensure_controller(pedicel_rig_data, controller_tool):
    """Creates the rotation root if this pedicel doesn't have one yet.
    Safe to call repeatedly - only acts once per pedicel."""
    if not pedicel_rig_data.controller_created:
        controller_tool.create_rotation_root(pedicel_rig_data)


# updated to check leaves as well
def check_against_all(rig, checker, pedicel, debug=False):
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
    """
    Reject-and-resample for a single pedicel: try a gaussian-sampled
    angle within its constraint range, accept if no collision, otherwise
    resample. Falls back to angle 0.0 if max_attempts is exhausted but checks to see is 0 deg is still
    valid in current environment.
    """
    ensure_controller(pedicel, controller_tool)

    min_a = pedicel.min_angle if pedicel.min_angle is not None else -20.0
    max_a = pedicel.max_angle if pedicel.max_angle is not None else 20.0

    # ignore this
    # best_angle = None
    # best_violation = float("inf")  # track least-bad rejected candidate, in case nothing clears

    for attempt in range(max_attempts):
        angle = sample_angle(min_a, max_a)

        if not validate_pedicel_angle(pedicel, angle):
            continue

        controller_tool.rotate(pedicel, angle)
        rejected, info = check_against_all(rig, checker, pedicel, debug=debug)

        if not rejected:
            if debug:
                print(f"  ACCEPTED {pedicel.prim.GetName()} @ {angle:.2f} deg (attempt {attempt + 1})")
            return True

        if debug:
            print(f"  reject {pedicel.prim.GetName()} @ {angle:.2f} deg (attempt {attempt + 1}): {info}")

    # All sampled angles rejected - check whether 0.0 is ACTUALLY safe against
    # current (possibly already-moved) neighbors, rather than assuming it is.
    controller_tool.rotate(pedicel, 0.0)
    rejected_at_rest, info = check_against_all(rig, checker, pedicel, debug=debug)

    if not rejected_at_rest:
        if debug:
            print(f"  FAILED to find valid pose for {pedicel.prim.GetName()} after {max_attempts} attempts - reset to 0.0 (verified safe)")
        return False

    # Even 0.0 is unsafe given current neighbor positions - this pedicel is
    # effectively boxed in. Flag it loudly rather than silently shipping a
    # colliding pose.
    print(f" {pedicel.prim.GetName()}: NO safe angle found, including 0.0 deg, "
          f"given current neighbor positions. {info}")
    return False


def randomize_all(rig, checker, controller_tool, max_attempts=20, seed=None, debug=False):
    """
    Top-level entry point. Applies default constraints if none are set,
    then randomizes every pedicel in sequence. Order matters: each
    pedicel's accepted pose is what later pedicels' collision checks
    see, so results are deterministic given a fixed seed but NOT
    independent of processing order.

    processiing order is randomized so that a pair that starts in contact doesn't
    have the same pedicel "fixed" first every time, which would bias the results. Variation is now more evenly spread
    """
    if seed is not None:
        random.seed(seed)
    
    # print(f"Seed: {seed}")

    apply_default_constraints(rig)  # no-op for pedicels that already have constraints set
    processing_order = list(rig.pedicels)
    random.shuffle(processing_order)
    results = {}
    for pedicel in processing_order:
        accepted = randomize_pedicel(rig, checker, controller_tool, pedicel, max_attempts=max_attempts, debug=debug)
        results[pedicel.prim.GetName()] = {
            "accepted": accepted,
            "final_angle": pedicel.current_angle,
        }

    return results
