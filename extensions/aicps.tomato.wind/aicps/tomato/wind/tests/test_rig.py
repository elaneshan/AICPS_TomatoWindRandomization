import omni.usd
import aicps.tomato.wind.rig as rig_module


def run():
    stage = omni.usd.get_context().get_stage()

    rig = rig_module.PlantRig(stage)
    rig.build()
    rig.summary()

    return rig  # handy if you want to inspect it further in the script editor

