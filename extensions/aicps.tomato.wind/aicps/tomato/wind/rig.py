from dataclasses import dataclass, field
from typing import List, Optional
from .constants import CONTROLLER_SUFFIX
from pxr import Usd, Gf, UsdGeom, Sdf  # Added Sdf import for path types
from . import transform as transform_module
from .registry import PlantRegistry
from .pivot_finder import PivotFinder


# Make data classes for the parts that we will be rotating in the cluster

@dataclass
class PedicelRigData:
    prim: Usd.Prim
    hinge_point: Gf.Vec3d = None
    affected_parts: List[Usd.Prim] = field(default_factory=list)
    original_parent_path: Optional[object] = None
    controller: Optional[Usd.Prim] = None
    controller_created: bool = False
    # Replaces current_angle - one entry per axis
    current_rotation: dict = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0}
    )
    # Replaces min_angle/max_angle - each axis independently
    # unconstrained (None) until set
    axis_limits: dict = field(
        default_factory=lambda: {
            "x": (None, None),
            "y": (None, None),
            "z": (None, None),
        }
    )


@dataclass
class LeafRigData:
    prim: Usd.Prim
    hinge_point: Gf.Vec3d = None
    original_parent_path: Optional[object] = None
    object_0_path: Sdf.Path = None  # Cached to skip subtree search on rebuild
    controller: Optional[Usd.Prim] = None
    controller_created: bool = False
    paired_pedicel_name: Optional[str] = None
    current_rotation: dict = field(
        default_factory=lambda: {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }
    )
    axis_limits: dict = field(
        default_factory=lambda: {
            "x": (0.0, 0.0),  # Keep X locked
            "y": (None, None),
            "z": (None, None),
        }
    )


class PlantRig:
    """
    Combines PlantRegistry discovery with PivotFinder hinge computation
    into a single rig-ready data structure. This is the checkpoint
    everything downstream (rig_builder, randomizer) will consume.
    """

    def __init__(self, stage, leaf_pairing_overrides=None):
        self.stage = stage
        self.registry = PlantRegistry(stage)
        self.pivot_finder = PivotFinder(stage)
        self.leaf_pairing_overrides = leaf_pairing_overrides or {}
        # {"foliage_leaf_01": "pedicel_01"}

        self.pedicels: List[PedicelRigData] = []
        self.leaves: List[LeafRigData] = []
        self._previous_leaf_data = {}
        self._previous_pedicel_data = {}

    def build(self):
        self.registry.build()

        # === FIX PART 1: Cache previous run's data mapped by NAME instead of PATH ===
        # This ensures lookups succeed even after prims are reparented under controllers
        self._previous_pedicel_data = {
            p.prim.GetName(): p for p in self.pedicels
        }
        self._previous_leaf_data = {
            l.prim.GetName(): l for l in self.leaves
        }

        self.pedicels.clear()
        self.leaves.clear()

        # Process pedicels
        for pedicel_data in self.registry.pedicels:
            affected_parts = list(pedicel_data.prim.GetChildren())
            original_parent_path = (
                pedicel_data.prim.GetPath().GetParentPath()
            )

            if transform_module.is_rigged(pedicel_data.prim):
                existing = self._previous_pedicel_data.get(
                    pedicel_data.prim.GetName()
                )
                hinge = (
                    existing.hinge_point
                    if existing
                    else self.pivot_finder.compute_hinge(pedicel_data)
                )
            else:
                hinge = self.pivot_finder.compute_hinge(pedicel_data)

            rig_data = PedicelRigData(
                prim=pedicel_data.prim,
                hinge_point=hinge,
                affected_parts=affected_parts,
                original_parent_path=original_parent_path,
            )

            if transform_module.is_rigged(pedicel_data.prim):
                rig_data.controller = pedicel_data.prim
                rig_data.controller_created = True
                rig_data.current_rotation = (
                    transform_module.get_current_rotation(
                        pedicel_data.prim
                    )
                )
                rig_data._original_op_order = (
                    transform_module.get_original_op_order(
                        pedicel_data.prim
                    )
                )

            self.pedicels.append(rig_data)

        # Process leaves
        for leaf_data in self.registry.leaves:
            original_parent_path = (
                leaf_data.prim.GetPath().GetParentPath()
            )

            if transform_module.is_rigged(leaf_data.prim):
                existing = self._previous_leaf_data.get(
                    leaf_data.prim.GetName()
                )
                hinge = (
                    existing.hinge_point
                    if existing
                    else self.pivot_finder.compute_leaf_hinge(
                        leaf_data.prim, None
                    )[0]
                )
                object_0_path = (
                    existing.object_0_path if existing else None
                )
            else:
                existing = self._previous_leaf_data.get(
                    leaf_data.prim.GetName()
                )
                cached_path = (
                    existing.object_0_path if existing else None
                )
                hinge, object_0_path = (
                    self.pivot_finder.compute_leaf_hinge(
                        leaf_data.prim, cached_path
                    )
                )

            rig_data = LeafRigData(
                prim=leaf_data.prim,
                hinge_point=hinge,
                original_parent_path=original_parent_path,
                object_0_path=object_0_path,
            )

            if transform_module.is_rigged(leaf_data.prim):
                rig_data.controller = leaf_data.prim
                rig_data.controller_created = True
                rig_data.current_rotation = (
                    transform_module.get_current_rotation(
                        leaf_data.prim
                    )
                )
                rig_data._original_op_order = (
                    transform_module.get_original_op_order(
                        leaf_data.prim
                    )
                )

            self.leaves.append(rig_data)

        # === Pairing: nearest pedicel by rest-pose hinge distance ===
        # Must reuse cached pairing once set, same rest-pose-invariant
        # reasoning as hinge_point (§3.3) — recomputing against a
        # mid-rotation pedicel's LIVE hinge would be wrong, but
        # hinge_point is already stored as the rest-pose value.
        for leaf_data in self.leaves:
            name = leaf_data.prim.GetName()

            if name in self.leaf_pairing_overrides:
                leaf_data.paired_pedicel_name = (
                    self.leaf_pairing_overrides[name]
                )
                continue

            existing = self._previous_leaf_data.get(name)

            if existing and existing.paired_pedicel_name:
                leaf_data.paired_pedicel_name = (
                    existing.paired_pedicel_name
                )
            elif self.pedicels:
                nearest = min(
                    self.pedicels,
                    key=lambda p: (
                        p.hinge_point - leaf_data.hinge_point
                    ).GetLength(),
                )
                leaf_data.paired_pedicel_name = nearest.prim.GetName()

    def summary(self):
        print("\n===== Plant Rig =====")
        print(f"Pedicels: {len(self.pedicels)}")

        for p in self.pedicels:
            print(
                f"  {p.prim.GetPath()}  "
                f"hinge={p.hinge_point}  "
                f"parts={len(p.affected_parts)}"
            )

        print(f"\nLeaves: {len(self.leaves)}")

        for l in self.leaves:
            print(
                f"  {l.prim.GetPath()}  "
                f"object_0_path={l.object_0_path}"
            )

        print("======================")