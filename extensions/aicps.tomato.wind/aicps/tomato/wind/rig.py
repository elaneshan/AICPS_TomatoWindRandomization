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
   current_rotation: dict = field(
       default_factory=lambda: {
           "x": 0.0,
           "y": 0.0,
           "z": 0.0,
       }
   )
   axis_limits: dict = field(
       default_factory=lambda: {
           "x": (0.0, 0.0),
           "y": (None, None),
           "z": (0.0, 0.0),
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

   def build(self):
       self.registry.build()

       # 1. Cache previous run's leaf data mapped by their path before clearing
       self._previous_leaf_data = {l.prim.GetPath(): l for l in self.leaves}

       self.pedicels.clear()
       self.leaves.clear()

       for pedicel_data in self.registry.pedicels:
           hinge = self.pivot_finder.compute_hinge(pedicel_data)
           affected_parts = list(pedicel_data.prim.GetChildren())
           original_parent_path = pedicel_data.prim.GetPath().GetParentPath()

           rig_data = PedicelRigData(
               prim=pedicel_data.prim,
               hinge_point=hinge,
               affected_parts=affected_parts,
               original_parent_path=original_parent_path,
           )

           # Reconcile with stage: if a controller already exists (from an
           # earlier rig/session), adopt it instead of assuming none exists.
           controller_path = original_parent_path.AppendChild(
               f"{pedicel_data.prim.GetName()}{CONTROLLER_SUFFIX}"
           )
           controller_prim = self.stage.GetPrimAtPath(controller_path)
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

       # Handle leaf data
       for leaf_data in self.registry.leaves:
           original_parent_path = leaf_data.prim.GetPath().GetParentPath()
           
           # 2. Check cache for this leaf prim's path to fetch its previously resolved object_0 path
           existing = self._previous_leaf_data.get(leaf_data.prim.GetPath())
           cached_path = existing.object_0_path if existing else None
           
           # 3. Calculate hinge and retrieve the found/cached object_0 path
           hinge, object_0_path = self.pivot_finder.compute_leaf_hinge(leaf_data.prim, cached_path)
           
           rig_data = LeafRigData(
               prim=leaf_data.prim,
               hinge_point=hinge,
               original_parent_path=original_parent_path,
               object_0_path=object_0_path, # Assign the resulting path back here
           )

           # Reconcile with stage: if a controller already exists (from an
           # earlier rig/session), adopt it instead of assuming none exists.
           controller_path = original_parent_path.AppendChild(
               f"{leaf_data.prim.GetName()}{CONTROLLER_SUFFIX}"
           )
           controller_prim = self.stage.GetPrimAtPath(controller_path)
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


   def summary(self):
       print("\n===== Plant Rig =====")
       print(f"Pedicels: {len(self.pedicels)}")
       for p in self.pedicels:
           print(f"  {p.prim.GetPath()}  hinge={p.hinge_point}  parts={len(p.affected_parts)}")

       print(f"\nLeaves: {len(self.leaves)}")
       for l in self.leaves:
           print(f"  {l.prim.GetPath()}  object_0_path={l.object_0_path}")
       print("======================")

