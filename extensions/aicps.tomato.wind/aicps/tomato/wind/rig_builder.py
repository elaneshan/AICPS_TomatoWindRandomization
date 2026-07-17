"""
DO NOT USE: depreciated
"""


from typing import List, Optional, Union


from pxr import Usd, UsdGeom, Gf, Sdf


from .constants import CONTROLLER_SUFFIX
from .rig import PedicelRigData, LeafRigData, PlantRig


RigItem = Union[PedicelRigData, LeafRigData]




class _RigBuildPlan:
    """Precomputed, stage-mutation-free build plan for a single rig item."""


    __slots__ = (
        "item",
        "parent_path",
        "controller_path",
        "controller_translate",
        "child_translate",
        "child_rotate",
        "child_scale",
    )


    def __init__(
        self,
        item: RigItem,
        parent_path: Sdf.Path,
        controller_path: Sdf.Path,
        controller_translate: Gf.Vec3d,
        child_translate: Gf.Vec3d,
        child_rotate: Gf.Vec3f,
        child_scale: Gf.Vec3f,
    ):
        self.item = item
        self.parent_path = parent_path
        self.controller_path = controller_path
        self.controller_translate = controller_translate
        self.child_translate = child_translate
        self.child_rotate = child_rotate
        self.child_scale = child_scale




class RigBuilder:
    """Builds physical Xform controllers on the USD stage from computed rig data."""


    def __init__(self, stage: Usd.Stage):
        self.stage = stage


    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------


    def build(self, plant_rig: PlantRig) -> None:
        """Build controllers for every pedicel and leaf tracked by plant_rig.


        Idempotent: items whose controller_created flag is already True
        (reconciled against an existing on-stage controller by PlantRig.build())
        are left untouched.
        """
        items: List[RigItem] = list(plant_rig.pedicels) + list(plant_rig.leaves)


        # --- Pass 1: validate + precompute all transforms, no stage mutation ---
        plans: List[_RigBuildPlan] = []
        for item in items:
            if item.controller_created:
                continue  # already rigged; skip to avoid double-compensating


            plan = self._precompute(item)
            if plan is not None:
                plans.append(plan)


        # --- Pass 2: mutate the stage ---
        for plan in plans:
            self._execute(plan)


    # ------------------------------------------------------------------
    # Precompute (read-only)
    # ------------------------------------------------------------------


    def _precompute(self, item: RigItem) -> Optional[_RigBuildPlan]:
        prim = item.prim
        hinge_point = item.hinge_point
        parent_path = item.original_parent_path


        if hinge_point is None:
            print(f"[RigBuilder] WARNING: no hinge_point for {prim.GetPath()}, skipping.")
            return None


        parent_prim = self.stage.GetPrimAtPath(parent_path)
        if not parent_prim.IsValid():
            print(f"[RigBuilder] WARNING: parent {parent_path} invalid for "
                  f"{prim.GetPath()}, skipping.")
            return None


        # Instancing-boundary guard. The prim we're about to reparent must be a
        # normal editable prim, not an instance proxy / prototype-internal prim.
        # Per the bug log, the instancing boundary sits at object_0, below the
        # foliage_leaf_XX Xform we operate on here -- so this should never fire
        # in practice. Fail loud instead of silently no-op'ing if it ever does.
        if prim.IsInstanceProxy() or prim.IsInPrototype():
            raise RuntimeError(
                f"[RigBuilder] {prim.GetPath()} is an instance proxy or lives "
                f"inside a prototype and cannot be reparented directly. Aborting "
                f"rig build for this prim."
            )


        controller_name = f"{prim.GetName()}{CONTROLLER_SUFFIX}"
        controller_path = parent_path.AppendChild(controller_name)


        if self.stage.GetPrimAtPath(controller_path).IsValid():
            # Shouldn't happen if PlantRig's reconciliation ran first, but guard
            # against stale/out-of-sync rig data clobbering an existing controller.
            print(f"[RigBuilder] WARNING: controller already exists at "
                  f"{controller_path} but item.controller_created was False; "
                  f"skipping to avoid clobbering.")
            return None


        # World matrices, computed BEFORE any mutation.
        parent_world = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        child_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )


        controller_world = Gf.Matrix4d(1.0)
        controller_world.SetTranslateOnly(hinge_point)


        # Controller's local transform = hinge point expressed in the parent's
        # local space.
        controller_local = controller_world * parent_world.GetInverse()
        controller_translate = controller_local.ExtractTranslation()


        # Child's new local transform under the controller: full decomposition,
        # not vector subtraction, so it's correct regardless of whatever
        # translate/rotate/scale the child already carries.
        child_local_under_controller = child_world * controller_world.GetInverse()
        child_translate, child_rotate, child_scale = self._decompose(
            child_local_under_controller
        )


        return _RigBuildPlan(
            item=item,
            parent_path=parent_path,
            controller_path=controller_path,
            controller_translate=controller_translate,
            child_translate=child_translate,
            child_rotate=child_rotate,
            child_scale=child_scale,
        )


    # ------------------------------------------------------------------
    # Execute (stage mutation)
    # ------------------------------------------------------------------


    def _execute(self, plan: _RigBuildPlan) -> None:
        item = plan.item
        prim = item.prim
        old_path = prim.GetPath()


        # 1. Create the controller Xform at the original parent path.
        controller_prim = UsdGeom.Xform.Define(self.stage, plan.controller_path).GetPrim()
        controller_xformable = UsdGeom.Xformable(controller_prim)
        controller_xformable.ClearXformOpOrder()
        controller_xformable.AddTranslateOp().Set(plan.controller_translate)
        # Identity rotate op, left empty for Phase 3 wind scripts to author onto.
        controller_xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))


        # 2. Reparent the original geometry under the new controller.
        new_path = plan.controller_path.AppendChild(prim.GetName())


        edit = Sdf.BatchNamespaceEdit()
        edit.Add(Sdf.NamespaceEdit.Reparent(old_path, plan.controller_path, -1))


        layer = self.stage.GetEditTarget().GetLayer()
        if not layer.Apply(edit):
            raise RuntimeError(
                f"[RigBuilder] Failed to reparent {old_path} -> {plan.controller_path}"
            )


        reparented_prim = self.stage.GetPrimAtPath(new_path)
        if not reparented_prim.IsValid():
            raise RuntimeError(f"[RigBuilder] Reparented prim not found at {new_path}")


        # 3. Compensate the reparented geometry's local transform so it stays
        #    exactly at its original world-space position/orientation.
        reparented_xformable = UsdGeom.Xformable(reparented_prim)
        reparented_xformable.ClearXformOpOrder()
        reparented_xformable.AddTranslateOp().Set(plan.child_translate)
        reparented_xformable.AddRotateXYZOp().Set(plan.child_rotate)
        if plan.child_scale != Gf.Vec3f(1.0, 1.0, 1.0):
            reparented_xformable.AddScaleOp().Set(plan.child_scale)


        # 4. Update rig-data bookkeeping so re-running PlantRig.build() will
        #    reconcile against this controller instead of re-creating it.
        item.prim = reparented_prim
        item.controller = controller_prim
        item.controller_created = True


    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------


    @staticmethod
    def _decompose(matrix: Gf.Matrix4d):
        """Decompose a Gf.Matrix4d into (translate, rotateXYZ degrees, scale).


        Axis order passed to Decompose() is reversed (Z, Y, X) to match
        UsdGeom's RotateXYZ op convention (rotation composed as Rx * Ry * Rz).
        Verify this against a known-good case before relying on it for
        anything with non-trivial authored rotation.
        """
        translate = matrix.ExtractTranslation()


        rotation = matrix.ExtractRotation()
        angles = rotation.Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis())
        rotate_xyz = Gf.Vec3f(angles[2], angles[1], angles[0])


        scale = Gf.Vec3f(
            Gf.Vec3d(matrix[0][0], matrix[0][1], matrix[0][2]).GetLength(),
            Gf.Vec3d(matrix[1][0], matrix[1][1], matrix[1][2]).GetLength(),
            Gf.Vec3d(matrix[2][0], matrix[2][1], matrix[2][2]).GetLength(),
        )


        return translate, rotate_xyz, scale



