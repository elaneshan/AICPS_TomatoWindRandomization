from pxr import UsdGeom
import omni.kit.commands
import omni.usd

from .constants import CONTROLLER_SUFFIX


class TransformController:
    """
    Builds a temporary rotation root ("controller") above a pedicel so it
    can be rotated around its hinge point without touching the pedicel's
    own (fragile, GLTF-imported) xformOp stack directly.

    State now lives on PedicelRigData itself (controller, controller_created,
    current_angle, original_parent_path) rather than in an internal dict here

    """
    
    def __init__(self, stage):
        self.stage = stage
        self._active = {}  # controller_path (str) -> original_parent_path

    def create_rotation_root(self, pedicel_rig_data):
        if pedicel_rig_data.controller_created:
            raise RuntimeError(
                f"{pedicel_rig_data.prim.GetPath()} already has an active controller. "
                f"Call reset() before creating a new one."
            )

        pedicel_prim = pedicel_rig_data.prim
        original_path = pedicel_prim.GetPath()
        original_parent_path = pedicel_rig_data.original_parent_path
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

        # we can just update rig data in place
        pedicel_rig_data.prim = self.stage.GetPrimAtPath(new_pedicel_path)
        pedicel_rig_data.controller = controller.GetPrim()
        pedicel_rig_data.controller_created = True
        pedicel_rig_data.current_angle = 0.0

        return pedicel_rig_data.controller

    def rotate(self, pedicel_rig_data, angle_degrees):
        if not pedicel_rig_data.controller_created:
            raise RuntimeError(
                f"{pedicel_rig_data.prim.GetPath()} has no active controller. "
                f"Call create_rotation_root() first."
            )

        xformable = UsdGeom.Xformable(pedicel_rig_data.controller)
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
                op.Set(angle_degrees)
                pedicel_rig_data.current_angle = angle_degrees
                return

        raise RuntimeError(f"No rotateY op found on {pedicel_rig_data.controller.GetPath()}")

    def reset(self, pedicel_rig_data):
        if not pedicel_rig_data.controller_created:
            print(f"{pedicel_rig_data.prim.GetPath()} has no active controller - nothing to reset.")
            return

        current_path = pedicel_rig_data.prim.GetPath()
        controller_path = pedicel_rig_data.controller.GetPath()
        original_parent_path = pedicel_rig_data.original_parent_path

        self.rotate(pedicel_rig_data, 0.0)

        restored_path = original_parent_path.AppendChild(pedicel_rig_data.prim.GetName())
        success, _ = omni.kit.commands.execute(
            "MovePrimCommand",
            path_from=str(current_path),
            path_to=str(restored_path),
            keep_world_transform=False,  # ops were never modified, just move the path back
            stage_or_context=self.stage,
        )
        if not success:
            raise RuntimeError(f"Failed to restore {current_path} to {restored_path}")

        self.stage.RemovePrim(controller_path)

        pedicel_rig_data.prim = self.stage.GetPrimAtPath(restored_path)
        pedicel_rig_data.controller = None
        pedicel_rig_data.controller_created = False
        pedicel_rig_data.current_angle = 0.0



_session = {}
def get_session():
    return _session
