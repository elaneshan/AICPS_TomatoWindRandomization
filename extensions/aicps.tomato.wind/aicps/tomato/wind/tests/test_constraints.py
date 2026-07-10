import omni.usd
import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.transform as transform_module
import aicps.tomato.wind.constraints as constraints_module


def run(pedicel_name="Pedicel_01"):
    stage = omni.usd.get_context().get_stage()

    rig = rig_module.PlantRig(stage)
    rig.build()

    constraints_module.apply_default_constraints(rig)

    print("Applied constraints:")
    for pedicel in rig.pedicels:
        print(f"  {pedicel.prim.GetName()}: [{pedicel.min_angle}, {pedicel.max_angle}]")

    pedicel = next(p for p in rig.pedicels if p.prim.GetName() == pedicel_name)

    controller_tool = transform_module.TransformController(stage)
    controller_tool.create_rotation_root(pedicel)

    print(f"\nTesting accept/reject on {pedicel_name}:")
    for test_angle in [3.0, 8.0, 15.0, -20.0]:
        accepted = constraints_module.try_rotate(controller_tool, pedicel, test_angle)
        print(f"  angle={test_angle:>6}  accepted={accepted}  current_angle={pedicel.current_angle}")

    controller_tool.reset(pedicel)
    print("\nReset complete.")


