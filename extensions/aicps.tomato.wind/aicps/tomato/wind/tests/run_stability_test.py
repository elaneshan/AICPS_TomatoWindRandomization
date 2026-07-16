import sys
import os
import time

# 1. SETUP SIMULATION APP FIRST (Crucial for standalone execution)
from isaacsim import SimulationApp

# Keep "headless": False so the Omniverse/Isaac Sim GUI pops up and renders
simulation_app = SimulationApp({"headless": True})

# 2. LATE IMPORTS (Must happen AFTER SimulationApp is initialized)
import omni.usd
import omni.kit.app
from omni.isaac.core.utils.stage import is_stage_loading

# Custom modules
import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.collisions as collisions_module
import aicps.tomato.wind.transform as transform
import aicps.tomato.wind.randomizer as randomizer

# 3. LOAD YOUR SPECIFIC USD FILE
USD_PATH = "/home/aicps/isaacsim_project/tomato_scene.usd"

print(f"Opening stage: {USD_PATH}")
omni.usd.get_context().open_stage(USD_PATH)

# Wait for Isaac Sim to finish loading the USD assets
while is_stage_loading():
    print("Waiting for USD stage to finish loading...")
    simulation_app.update()

# 4. INITIALIZE SIMULATION SCENE & RIGS
stage = omni.usd.get_context().get_stage()
rig = rig_module.PlantRig(stage)
rig.build()

controller = transform.TransformController(stage)
trellis_prim = stage.GetPrimAtPath("/World/Trellis")
leaf_prims = [l.prim for l in rig.leaves]
checker = collisions_module.CollisionChecker(stage, environment_prim=trellis_prim, leaf_prims=leaf_prims)
checker.capture_baselines(rig)

# 5. RUN THE STABILITY RUN
seeds_to_test = list(range(1, 21))  # 20 seeds
failures = []
unclean_baselines = []
all_results = {}

print(f"Starting {len(seeds_to_test)}-seed stability run...")
start_time = time.time()

for current_seed in seeds_to_test:
    results = randomizer.randomize_all(rig, checker, controller, seed=current_seed, debug=False)
    all_results[current_seed] = results

    for name, data in results.items():
        if not data["accepted"]:
            failures.append((current_seed, name))

    has_rejects = checker.baseline_report(rig, debug=False)
    if has_rejects:
        unclean_baselines.append(current_seed)

    # Keep the simulator and GUI ticking
    simulation_app.update()

elapsed = time.time() - start_time
print(f"\n===== STABILITY SUMMARY ({len(seeds_to_test)} seeds, {elapsed:.1f}s) =====")
print(f"Fallback-to-0deg failures: {failures if failures else 'none'}")
print(f"Seeds with unclean post-randomization baseline: {unclean_baselines if unclean_baselines else 'none'}")
print("=====================================\n")

# 6. CLEAN EXIT
simulation_app.close()

