"""
capture.py -- semantic labeling + Replicator-based dataset capture.


Classes for the batch: fruit_ripe, fruit_partial, fruit_unripe, pedicel, leaf.
 - fruit_*  -> each pedicel's tomato child, split by ripeness from the prim name
 - pedicel  -> each pedicel's stem segments (segment_A/segment_B) - both
               segments share the same "pedicel" label, so they collapse
               into one class in the SEMANTIC output (instance output will
               still show them as separate instances - that's expected)
 - leaf     -> each leaf prim
 -rachis    -> the main rachis stem
"""
import os
import omni.replicator.core as rep
from isaacsim.core.utils.semantics import add_labels
from . import camera as camera_module
from . import collisions as collisions_module
from . import scene as scene_module


def _fruit_class(tomato_prim):
    """Maps a tomato prim's name to a ripeness class. Source assets are
    named Tomato_ripe_*, Tomato_partial_*, Tomato_unripe_* (casing is
    inconsistent in the scene, hence the .lower()). "unripe" MUST be
    checked before "ripe" - "unripe" contains "ripe" as a substring, so
    checking "ripe" first would silently misclassify every unripe tomato
    as ripe."""
    name = tomato_prim.GetName().lower()
    if "unripe" in name:
        return "fruit_unripe"
    elif "partial" in name:
        return "fruit_partial"
    elif "ripe" in name:
        return "fruit_ripe"
    else:
        print(f"WARNING: could not determine ripeness for {tomato_prim.GetPath()}, defaulting to fruit_ripe")
        return "fruit_ripe"


def label_scene_for_segmentation(rig):
    """Applies semantic class labels to the prims that matter for training.
    Safe to call more than once (add_labels overwrites, doesn't stack)."""
    fruit_counts = {"fruit_ripe": 0, "fruit_partial": 0, "fruit_unripe": 0}
    pedicel_count = leaf_count = 0
    for pedicel in rig.pedicels:
        tomato = collisions_module.find_child_by_prefix(pedicel, "tomato")
        if tomato is not None:
            fruit_class = _fruit_class(tomato)
            add_labels(tomato, labels=[fruit_class], instance_name="class")
            fruit_counts[fruit_class] += 1
        for seg in collisions_module.find_stem_segments(pedicel):
            add_labels(seg, labels=["pedicel"], instance_name="class")
            pedicel_count += 1
    for leaf in rig.leaves:
        add_labels(leaf.prim, labels=["leaf"], instance_name="class")
        leaf_count += 1

    # rachis: add here, e.g.
    rachis_prim = rig.stage.GetPrimAtPath("/World/Tomato_Cluster_Assembly/Peduncle/Rachis")
    if rachis_prim.IsValid():
        add_labels(rachis_prim, labels=["rachis"], instance_name="class")

    print(
        f"Labeled: {fruit_counts['fruit_ripe']} ripe, {fruit_counts['fruit_partial']} partial, "
        f"{fruit_counts['fruit_unripe']} unripe fruit | {pedicel_count} pedicel segments | {leaf_count} leaves"
    )


async def generate_labeled_batch(
    stage,
    rig,
    checker,
    controller_tool,
    output_dir,
    n=10,
    seeds=None,
    max_attempts=20,
    camera_kwargs=None,
    lighting_kwargs=None,
    resolution=(1280, 720),
    capture_depth=True,
    capture_segmentation=True,
    capture_instances=True,
    rt_subframes=32,
    debug=False,
):
    """Runs randomize_scene() n times and writes RGB (+depth, +segmentation)
    to output_dir via Replicator's BasicWriter. One frame per seed."""
    label_scene_for_segmentation(rig)
    camera_prim = stage.GetPrimAtPath(camera_module.CAMERA_PATH)
    if not camera_prim.IsValid():
        camera_module.create_overview_camera(stage)
    render_product = rep.create.render_product(camera_module.CAMERA_PATH, resolution)
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        output_dir=output_dir,
        rgb=True,
        distance_to_camera=capture_depth,
        semantic_segmentation=capture_segmentation,
        colorize_semantic_segmentation=True,
        instance_segmentation=capture_instances,
        colorize_instance_segmentation=True,
    )
    writer.attach([render_product])
    if seeds is None:
        seeds = list(range(n))
    else:
        n = len(seeds)
    os.makedirs(output_dir, exist_ok=True)
    print(f"\nCapturing {n} frames to {output_dir}")
    frames = []
    for i, seed in enumerate(seeds):
        result = scene_module.randomize_scene(
            stage, rig, checker, controller_tool,
            camera_kwargs=camera_kwargs, lighting_kwargs=lighting_kwargs,
            max_attempts=max_attempts, seed=seed, debug=debug,
        )
        await rep.orchestrator.step_async(rt_subframes=rt_subframes, pause_timeline=False)
        frames.append(result)
        cam = result["camera"]
        print(
            f"[{i+1:>2}/{n}] seed={seed:<6} captured "
            f"(fruit_visible={cam['fruit_visible']}, "
            f"pose_failed={result['pose_failed_count']})"
        )
    await rep.orchestrator.wait_until_complete_async()
    print(f"\nDone. {n} frames written to {output_dir}")
    return frames

