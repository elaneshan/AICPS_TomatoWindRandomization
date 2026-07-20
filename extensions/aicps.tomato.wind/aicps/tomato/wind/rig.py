from dataclasses import dataclass, field
from typing import List, Optional
from .constants import CONTROLLER_SUFFIX
from pxr import Usd, Gf, UsdGeom, Sdf  # Added Sdf import for path types

from .registry import PlantRegistry
from .pivot_finder import PivotFinder

# make data classes for the parts that we will be rotating in the cluster

@dataclass
class PedicelRigData:
  prim: Usd.Prim
  hinge_point: Gf.Vec3d = None
  affected_parts: List[Usd.Prim] = field(default_factory=list)
  original_parent_path: Optional[object] = None
  controller: Optional[Usd.Prim] = None
  controller_created: bool = False
  # replaces current_angle - one entry per axis
  current_rotation: dict = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})
  # replaces min_angle/max_angle - each axis independently unconstrained (None) until set
  axis_limits: dict = field(default_factory=lambda: {
      "x": (None, None), "y": (None, None), "z": (None, None)
  })


@dataclass
class LeafRigData:
  prim: Usd.Prim
  hinge_point: Gf.Vec3d = None
  original_parent_path: Optional[object] = None
  object_0_path: Sdf.Path = None   # new -- cached to skip subtree search on rebuild
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
          "x": (0.0, 0.0), #keep X locked
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

  def __init__(self, stage):
      self.stage = stage
      self.registry = PlantRegistry(stage)
      self.pivot_finder = PivotFinder(stage)

      self.pedicels: List[PedicelRigData] = []
      self.leaves: List[LeafRigData] = []
      self._previous_leaf_data = {}  # Tracks leaf properties across rebuilds
      self._previous_pedicel_data = {} # Tracks pedicel properties across rebuilds

  def build(self):
      self.registry.build()

      # === FIX PART 1: Cache previous run's data mapped by NAME instead of PATH ===
      # This ensures lookups succeed even after prims are reparented under controllers
      self._previous_pedicel_data = {p.prim.GetName(): p for p in self.pedicels}
      self._previous_leaf_data = {l.prim.GetName(): l for l in self.leaves}

      self.pedicels.clear()
      self.leaves.clear()

      # Process pedicels
      for pedicel_data in self.registry.pedicels:
          affected_parts = list(pedicel_data.prim.GetChildren())
          parent_prim = pedicel_data.prim.GetParent()
          
          # === FIX PART 2: Check reconciliation first before computing anything ===
          if parent_prim.GetName().endswith(CONTROLLER_SUFFIX):
              controller_prim = parent_prim
              original_parent_path = controller_prim.GetPath().GetParentPath()
              
              # PROTECT REST POSE INVARIANT: Short-circuit and use cache if available
              existing = self._previous_pedicel_data.get(pedicel_data.prim.GetName())
              hinge = existing.hinge_point if existing else self.pivot_finder.compute_hinge(pedicel_data)
          else:
              original_parent_path = pedicel_data.prim.GetPath().GetParentPath()
              controller_path = original_parent_path.AppendChild(
                  f"{pedicel_data.prim.GetName()}{CONTROLLER_SUFFIX}"
              )
              controller_prim = self.stage.GetPrimAtPath(controller_path)
              hinge = self.pivot_finder.compute_hinge(pedicel_data)

          rig_data = PedicelRigData(
              prim=pedicel_data.prim,
              hinge_point=hinge,
              affected_parts=affected_parts,
              original_parent_path=original_parent_path,
          )

          if controller_prim.IsValid():
              rig_data.controller = controller_prim
              rig_data.controller_created = True
              xformable = UsdGeom.Xformable(controller_prim)
              for op in xformable.GetOrderedXformOps():
                  if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                      x, y, z = op.Get()
                      rig_data.current_rotation = {"x": x, "y": y, "z": z}
                      break

          self.pedicels.append(rig_data)

      # Process leaves
      for leaf_data in self.registry.leaves:
          parent_prim = leaf_data.prim.GetParent()
          
          # === FIX PART 3: Reconciliation check first for leaves ===
          if parent_prim.GetName().endswith(CONTROLLER_SUFFIX):
              controller_prim = parent_prim
              original_parent_path = controller_prim.GetPath().GetParentPath()
              
              # PROTECT REST POSE INVARIANT: Don't read live geometry if already rigged
              existing = self._previous_leaf_data.get(leaf_data.prim.GetName())
              hinge = existing.hinge_point if existing else self.pivot_finder.compute_leaf_hinge(leaf_data.prim, None)[0]
              object_0_path = existing.object_0_path if existing else None
          else:
              original_parent_path = leaf_data.prim.GetPath().GetParentPath()
              controller_path = original_parent_path.AppendChild(
                  f"{leaf_data.prim.GetName()}{CONTROLLER_SUFFIX}"
              )
              controller_prim = self.stage.GetPrimAtPath(controller_path)
              
              existing = self._previous_leaf_data.get(leaf_data.prim.GetName())
              cached_path = existing.object_0_path if existing else None
              hinge, object_0_path = self.pivot_finder.compute_leaf_hinge(leaf_data.prim, cached_path)

          rig_data = LeafRigData(
              prim=leaf_data.prim,
              hinge_point=hinge,
              original_parent_path=original_parent_path,
              object_0_path=object_0_path,
          )

          if controller_prim.IsValid():
              rig_data.controller = controller_prim
              rig_data.controller_created = True
              xformable = UsdGeom.Xformable(controller_prim)
              for op in xformable.GetOrderedXformOps():
                  if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                      x, y, z = op.Get()
                      rig_data.current_rotation = {"x": x, "y": y, "z": z}
                      break

          self.leaves.append(rig_data)
      # === Pairing: nearest pedicel by rest-pose hinge distance ===
      # Must reuse cached pairing once set, same rest-pose-invariant reasoning
      # as hinge_point (§3.3) — recomputing against a mid-rotation pedicel's
      # LIVE hinge would be wrong, but hinge_point is already stored as the
      # rest-pose value, so this is safe as long as it's computed from
      # hinge_point (never from live geometry) and cached like everything else.
      for leaf_data in self.leaves:
        existing = self._previous_leaf_data.get(leaf_data.prim.GetName())
        if existing and existing.paired_pedicel_name:
            leaf_data.paired_pedicel_name = existing.paired_pedicel_name
        elif self.pedicels:
            nearest = min(
                self.pedicels,
                key=lambda p: (p.hinge_point - leaf_data.hinge_point).GetLength(),
            )
            leaf_data.paired_pedicel_name = nearest.prim.GetName()




  def summary(self):
      print("\n===== Plant Rig =====")
      print(f"Pedicels: {len(self.pedicels)}")
      for p in self.pedicels:
          print(f"  {p.prim.GetPath()}  hinge={p.hinge_point}  parts={len(p.affected_parts)}")

      print(f"\nLeaves: {len(self.leaves)}")
      for l in self.leaves:
          print(f"  {l.prim.GetPath()}  object_0_path={l.object_0_path}")
      print("======================")

