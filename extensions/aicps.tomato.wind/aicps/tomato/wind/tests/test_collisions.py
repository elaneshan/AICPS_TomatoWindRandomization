import omni.usd
import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.transform as transform_module
import aicps.tomato.wind.constraints as constraints_module
import aicps.tomato.wind.collisions as collisions_module


def run(pedicel_name="Pedicel_01", test_angle=8.0):
    stage = omni.usd.get_context().get_stage()

    rig = rig_module.PlantRig(stage)
    rig.build()
    constraints_module.apply_default_constraints(rig)

    checker = collisions_module.CollisionChecker(stage)

    # Sanity check FIRST, before rotating anything.
    has_baseline_overlap = checker.baseline_report(rig, debug=True)
    if has_baseline_overlap:
        print("NOTE: baseline overlaps exist - collision rejection below may "
              "trigger even on angles that look visually fine. Investigate "
              "before trusting reject results.")
