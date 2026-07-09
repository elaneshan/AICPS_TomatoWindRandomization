from dataclasses import dataclass, field
from typing import List, Optional

from pxr import Usd, Gf

from .registry import PlantRegistry
from .pivot_finder import PivotFinder

# make data classes for the parts that we will be rotating in the cluster

@dataclass
class PedicelRigData:
    prim: Usd.Prim
    hinge_point: Gf.Vec3d = None
    # TODO: derive from geometry (e.g. normalize(rachis_connection - hinge_point))
    # Currently hardcoded to world Y based on manual test of a single pedicel
    # (Pedicel_Test_Root). Not yet validated across Pedicel_01–08 or differing
    # orientations around the rachis. Revisit once rotation testing begins —
    # if any pedicel rotates sideways, this needs to become computed per-prim.
    rotation_axis: Gf.Vec3d = field(default_factory=lambda: Gf.Vec3d(0, 1, 0))
    affected_parts: List[Usd.Prim] = field(default_factory=list)

@dataclass
class LeafRigData:
    prim: Usd.Prim
    pivot_prim: Usd.Prim = None
    parent_relationship: Optional[Usd.Prim] = None


class PlantRig:
    """
    Combines PlantRegistry discovery with PivotFinder hinge computation
    into a single rig-ready data structure. This is the checkpoint
    everything downstream (rig_builder, randomizer) will consume.
    """

    def __init__(self, stage):
        self.stage = stage
        self.registry = PlantRegistry(stage)
        self.pivot_finder = PivotFinder(stage)

        self.pedicels: List[PedicelRigData] = []
        self.leaves: List[LeafRigData] = []

    def build(self):
        self.registry.build()

        self.pedicels.clear()
        self.leaves.clear()

        for pedicel_data in self.registry.pedicels: # get all of the pedicels
            hinge = self.pivot_finder.compute_hinge(pedicel_data) # for each one we compute the hinge

            # Everything directly under the Pedicel moves with it:
            # tomato, calyx, segment_A, segment_B.
            affected_parts = list(pedicel_data.prim.GetChildren())

            self.pedicels.append(
                PedicelRigData(
                    prim=pedicel_data.prim,
                    hinge_point=hinge,
                    affected_parts=affected_parts,
                )
            )

        for leaf_data in self.registry.leaves:
            self.leaves.append(
                LeafRigData(
                    prim=leaf_data.prim,
                    pivot_prim=leaf_data.prim,
                    parent_relationship=leaf_data.prim.GetParent(),
                )
            )

    def summary(self):
        print("\n===== Plant Rig =====")
        print(f"Pedicels: {len(self.pedicels)}")
        for p in self.pedicels:
            print(f"  {p.prim.GetPath()}  hinge={p.hinge_point}  parts={len(p.affected_parts)}")

        print(f"\nLeaves: {len(self.leaves)}")
        for l in self.leaves:
            print(f"  {l.prim.GetPath()}")
        print("======================")
