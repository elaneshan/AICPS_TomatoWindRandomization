import omni.usd

# from aicps.tomato.wind.registry import PlantRegistry
import aicps.tomato.wind.registry as Registry


def run():

    stage = omni.usd.get_context().get_stage()

    registry = Registry.PlantRegistry(stage)
    registry.build()

    registry.summary()

    print("\nPedicels:")
    for pedicel in registry.pedicels:
        print(" ", pedicel.prim.GetPath())

    print("\nLeaves:")
    for leaf in registry.leaves:
        print(" ", leaf.prim.GetPath())

