import omni.usd

from aicps.tomato.wind.registry import PlantRegistry
from aicps.tomato.wind.pivot_finder import PivotFinder


def run():

    stage = omni.usd.get_context().get_stage()

    registry = PlantRegistry(stage)
    registry.build()

    finder = PivotFinder(stage)

    print()

    for pedicel in registry.pedicels:

        hinge = finder.compute_hinge(pedicel)

        print(pedicel.prim.GetName())
        print(" ", hinge)
