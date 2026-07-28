# run directly on isaac sim script editor (currently n=10 frames)
"""
Full pipeline demo -- run this on a FRESH stage with the tomato cluster
loaded (nothing rigged/rotated yet). Builds the rig, captures collision
baselines, then randomizes pedicel/leaf wind pose + camera + lighting for
each frame and writes RGB + depth + semantic/instance segmentation to disk.
"""
import asyncio
import omni.usd as usd
import aicps.tomato.wind.collisions as collisions
import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.transform as transform
import aicps.tomato.wind.capture as capture

# 1. Grab stage context
stage = usd.get_context().get_stage()

# 2. Leaf pairing overrides - verified against the viewport, do not trust
#    manual hinge defaults
leaf_pairing_overrides = {
    "foliage_leaf_01": "Pedicel_01",
    "foliage_leaf_07": "Pedicel_01",
    "foliage_leaf_05": "Pedicel_02",
    "foliage_leaf_02": "Pedicel_04",
    "foliage_leaf_06": "Pedicel_08",
    "foliage_leaf_03": "Pedicel_06",
    "foliage_leaf_04": "Pedicel_05",
}

# 3. Build rig, checker, controller
rig = rig_module.PlantRig(stage, leaf_pairing_overrides=leaf_pairing_overrides)
rig.build()

checker = collisions.CollisionChecker(stage, leaf_rig_items=rig.leaves)
checker.capture_baselines(rig)
print("Baselines captured.")

controller_tool = transform.TransformController(stage)

# 4. Capture the labeled batch -- writes RGB + depth + colorized semantic/
#    instance segmentation masks (fruit_ripe/fruit_partial/fruit_unripe/
#    pedicel/leaf) to disk. One frame per seed, cluster pose + camera +
#    lighting all randomized together per frame via randomize_scene().
async def _run():
    frames = await capture.generate_labeled_batch(
        stage, rig, checker, controller_tool,
        output_dir="/home/aicps/isaacsim_project/tomato_pi_demo_batch",  # change if you want it elsewhere
        n=10,
        max_attempts=20,
        camera_kwargs={"elevation_range": (-25.0, 65.0)},
    )
    return frames

asyncio.ensure_future(_run())

