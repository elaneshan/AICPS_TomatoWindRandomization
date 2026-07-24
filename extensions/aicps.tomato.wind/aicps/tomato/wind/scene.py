"""
scene.py -- combines cluster pose (randomizer.py), camera (camera.py), and
lighting (lighting.py) into one seeded call
"""

import random

from . import camera as camera_module
from . import lighting as lighting_module
from . import randomizer as randomizer_module


def focus_overview_camera(camera_path=camera_module.CAMERA_PATH):
    """Points the active viewport at OverviewCam so we can see what the
    randomized camera pose actually looks like"""
    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport is None:
        print("WARNING: no active viewport found")
        return
    viewport.camera_path = camera_path
    print(f"Viewport now looking through {camera_path}")


def randomize_scene(
    stage,
    rig,
    checker,
    controller_tool,
    camera_kwargs=None,
    lighting_kwargs=None,
    max_attempts=20,
    seed=None,
    debug=False,
):
    """One-call chain: cluster pose -> camera -> lighting.

    Order matters:
      1. Cluster pose first (randomizer.randomize_all) - collision-constrained,
         the slowest and most failure-prone step, and everything downstream
         (fruit visibility for the camera gate) depends on final geometry.
      2. Camera second - randomize_overview_camera_with_fruit() calls
         get_cluster_bounds()/compute_front_azimuth() fresh, so it needs the
         cluster's FINAL pose, not the rest pose.
      3. Lighting last - independent of geometry, but kept in this same
         seeded call so one seed reproduces the whole scene, not just the pose.

    Seeding note: randomize_all() calls random.seed(seed) internally when
    given a seed. So seed ONCE at the top of this function and pass seed=None
    down to randomize_all() so it continues on the same stream instead of
    resetting it.
    """
    if seed is not None:
        random.seed(seed)

    camera_kwargs = dict(camera_kwargs or {})
    lighting_kwargs = dict(lighting_kwargs or {})

    pose_results = randomizer_module.randomize_all(
        rig, checker, controller_tool,
        max_attempts=max_attempts, seed=None, debug=debug,
    )

    camera_info = camera_module.randomize_overview_camera_with_fruit(
        stage, **camera_kwargs
    )
    lighting_info = lighting_module.randomize_lighting(stage, **lighting_kwargs)

    failed = {name: r for name, r in pose_results.items() if not r["accepted"]}

    return {
        "seed": seed,
        "pose_results": pose_results,
        "pose_failed_count": len(failed),
        "pose_failed_names": list(failed.keys()),
        "camera": camera_info,
        "lighting": lighting_info,
    }


def generate_baby_batch(
    stage,
    rig,
    checker,
    controller_tool,
    n=10,
    seeds=None,
    max_attempts=20,
    camera_kwargs=None,
    lighting_kwargs=None,
    debug=False,
):
    """Runs randomize_scene() n times and prints a per-frame summary table
    NOTE: this does NOT write frames to disk. 
    """
    if seeds is None:
        seeds = list(range(n))
    else:
        n = len(seeds)

    print("\n" + "=" * 70)
    print(f"BABY BATCH: {n} scenes")
    print("=" * 70)

    frames = []
    for i, seed in enumerate(seeds):
        result = randomize_scene(
            stage, rig, checker, controller_tool,
            camera_kwargs=camera_kwargs, lighting_kwargs=lighting_kwargs,
            max_attempts=max_attempts, seed=seed, debug=debug,
        )
        frames.append(result)

        cam = result["camera"]
        light = result["lighting"]
        print(
            f"[{i+1:>2}/{n}] seed={seed:<6} "
            f"pose_failed={result['pose_failed_count']:<2} "
            f"az={cam['azimuth_deg']:6.1f} el={cam['elevation_deg']:6.1f} "
            f"dist={cam['distance']:.2f} fruit_visible={cam['fruit_visible']} "
            f"(cam attempts={cam['attempts']}) | "
            f"dome={light['dome_intensity']:.0f}@{light['dome_color_temp']:.0f}K "
            f"key={light['key_intensity']:.0f}@{light['key_color_temp']:.0f}K "
            f"el={light['key_elevation_deg']:.1f}"
        )
        if result["pose_failed_count"] > 0:
            print(f"        pose fallbacks: {result['pose_failed_names']}")

    print("-" * 70)
    no_fruit = [f for f in frames if not f["camera"]["fruit_visible"]]
    any_pose_fail = [f for f in frames if f["pose_failed_count"] > 0]
    print(f"Frames with NO visible fruit after max_tries: {len(no_fruit)}/{n}")
    print(f"Frames with >=1 pose fallback:                {len(any_pose_fail)}/{n}")
    print("=" * 70 + "\n")

    return frames

