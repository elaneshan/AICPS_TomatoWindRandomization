import omni.usd
import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.transform as transform_module


def start(pedicel_index=0, angle=5):
    stage = omni.usd.get_context().get_stage()

    rig = rig_module.PlantRig(stage)
    rig.build()

    controller_tool = transform_module.TransformController(stage)
    pedicel = rig.pedicels[pedicel_index]

    print(f"Testing rotation on {pedicel.prim.GetPath()}")
    print(f"Hinge: {pedicel.hinge_point}")

    controller = controller_tool.create_rotation_root(pedicel)
    print(f"Controller created: {controller.GetPath()}")

    controller_tool.rotate(controller, angle)
    print(f"Rotated {angle} degrees. Inspect viewport now.")

    session = transform_module.get_session()
    session["controller_tool"] = controller_tool
    session["pedicel"] = pedicel


def finish():
    session = transform_module.get_session()
    if "controller_tool" not in session:
        print("Nothing active — run start() first.")
        return

    session["controller_tool"].reset(session["pedicel"])
    session.clear()
    print("Reset complete.")

