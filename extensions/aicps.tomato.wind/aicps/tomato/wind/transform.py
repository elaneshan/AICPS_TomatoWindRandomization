from pxr import UsdGeom
import omni.kit.commands
import omni.usd

from .constants import CONTROLLER_SUFFIX


class TransformController:
    def __init__(self, stage):
        self.stage = stage
        self._active = {}  # controller_path (str) -> original_parent_path

    def create_rotation_root(self, pedicel_rig_data):
        pedicel_prim = pedicel_rig_data.prim
        original_path = pedicel_prim.GetPath()
        original_parent_path = original_path.GetParentPath()
        pedicel_name = pedicel_prim.GetName()

        controller_path = original_parent_path.AppendChild(
            f"{pedicel_name}{CONTROLLER_SUFFIX}"
        )

        # Always re-fetch prims fresh from the stage - never reuse a cached
        # Usd.Prim reference across runs, it can go stale after edits.
        parent_prim = self.stage.GetPrimAtPath(original_parent_path)
        parent_world = omni.usd.get_world_transform_matrix(parent_prim)
        hinge_local = parent_world.GetInverse().Transform(pedicel_rig_data.hinge_point)

        controller = UsdGeom.Xform.Define(self.stage, controller_path)
        xformable = UsdGeom.Xformable(controller)
        xformable.ClearXformOpOrder()

        # Standard pivot pattern: translate to pivot, rotate, translate back.
        # At angle=0 this whole stack collapses to identity.
        xformable.AddTranslateOp(opSuffix="pivot").Set(hinge_local) # move to the hinge
        xformable.AddRotateYOp() # rotate it
        xformable.AddTranslateOp(opSuffix="pivot", isInverseOp=True) # move back

        new_pedicel_path = controller_path.AppendChild(pedicel_name)
        success, _ = omni.kit.commands.execute(
            "MovePrimCommand",
            path_from=str(original_path),
            path_to=str(new_pedicel_path),
            keep_world_transform=False,  # controller is identity at rest - no compensation needed
            stage_or_context=self.stage,
        )
        if not success:
            raise RuntimeError(f"Failed to reparent {original_path} under {controller_path}")

        pedicel_rig_data.prim = self.stage.GetPrimAtPath(new_pedicel_path)
        self._active[str(controller_path)] = original_parent_path
        return controller.GetPrim()

    def rotate(self, controller_prim, angle_degrees):
        xformable = UsdGeom.Xformable(controller_prim)
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
                op.Set(angle_degrees)
                return
        raise RuntimeError(f"No rotateY op found on {controller_prim.GetPath()}")

    def reset(self, pedicel_rig_data):
        current_path = pedicel_rig_data.prim.GetPath()
        controller_path = current_path.GetParentPath()
        original_parent_path = self._active.get(str(controller_path))
        if original_parent_path is None:
            raise RuntimeError(f"No active controller found for {current_path}")

        self.rotate(self.stage.GetPrimAtPath(controller_path), 0.0)

        restored_path = original_parent_path.AppendChild(pedicel_rig_data.prim.GetName())
        omni.kit.commands.execute(
            "MovePrimCommand",
            path_from=str(current_path),
            path_to=str(restored_path),
            keep_world_transform=False,  # ops never changed, just move the path back
            stage_or_context=self.stage,
        )

        self.stage.RemovePrim(controller_path)
        pedicel_rig_data.prim = self.stage.GetPrimAtPath(restored_path)
        self._active.pop(str(controller_path), None)

_session = {}
def get_session():
    return _session
