from pxr import UsdGeom, Gf
import omni.usd


# --- v17 "Option 1" -----------------------------------------------------
# This suffix tags the ops WE author, distinct from any op already on the
# pedicel from its GLTF import -- confirmed this session that every
# pedicel already has its OWN 'xformOp:translate:pivot' from that import
# (a different point than our computed hinge). Using the same suffix
# would either collide outright or silently clobber the GLTF's real
# pivot. Do not rename this to "pivot".
#
# TODO (next edit, constants.py): this should replace CONTROLLER_SUFFIX
# there -- its meaning has shifted from "a sibling prim's name suffix"
# to "the tag distinguishing our injected ops from the pedicel's
# original ones" (v17 handoff SS4). Defined locally here for now so this
# file is self-contained until that rename happens.
from .constants import ROTATION_OP_SUFFIX
_TRANSLATE_ATTR_NAME = f"xformOp:translate:{ROTATION_OP_SUFFIX}_pivot"
_ROTATE_ATTR_NAME = f"xformOp:rotateXYZ:{ROTATION_OP_SUFFIX}"


class TransformController:
    """
    Rotates a pedicel around its hinge point by authoring three ops
    DIRECTLY on the pedicel's own prim -- translate-to-hinge, rotate,
    inverse-translate -- prepended ahead of its existing xformOpOrder.

    REPLACES the old sibling-controller-prim + MovePrimCommand approach
    (v8-v16). That approach reparented each pedicel under a new "rotation
    root" Xform -- which turned out to be structurally broken (v17
    handoff): the cluster is referenced into the scene (cluster_lvl_1.usd
    -> cr3_scene.usd), and USD does not allow renaming/reparenting an
    ancestral prim, full stop, regardless of edit target. Confirmed via
    direct MovePrimCommand test on a real pedicel, isolated from all
    rigging code, this session.

    Mathematically equivalent to the old approach (same composition:
    parent_world x (T(hinge)*R*T(-hinge)) x pedicel_own_ops, just
    authored on one prim instead of two) -- validated empirically via
    smoke_test_inplace_rotation.py this session, including an identity
    check (angle=0 must exactly reproduce the pre-edit world transform)
    and a full rotate/reset round-trip.

    IMPORTANT, learned the hard way writing the smoke test: Add*Op
    authors onto the USD stage IMMEDIATELY -- it is NOT rolled back if a
    later line raises. A crash between create_rotation_root() and
    reset() leaves real ops sitting on the prim. create_rotation_root()
    below defends against this (cleans up detected leftovers from a
    prior incomplete run before proceeding) rather than crashing on a
    name collision.

    State still lives on PedicelRigData (rig.py) -- controller_created,
    current_rotation, as before. One NEW piece of state is required:
    the pedicel's ORIGINAL xformOpOrder (captured before we touch it),
    so reset() can restore it exactly rather than guessing. Stored as a
    dynamically-added attribute (_original_op_order) for now -- TODO:
    promote to a proper PedicelRigData field next time rig.py is edited,
    this is a stopgap so this change stays scoped to transform.py alone.
    """

    def __init__(self, stage):
        self.stage = stage

    def create_rotation_root(self, pedicel_rig_data):
        if pedicel_rig_data.controller_created:
            raise RuntimeError(
                f"{pedicel_rig_data.prim.GetPath()} already has an active rotation rig. "
                f"Call reset() before creating a new one."
            )

        prim = pedicel_rig_data.prim
        xformable = UsdGeom.Xformable(prim)

        # --- defend against leftover ops from a PREVIOUS crashed run ---
        # controller_created is False here (checked above), but a crash
        # AFTER Add*Op calls and BEFORE controller_created got set to
        # True (further down) would leave real, already-authored ops on
        # this exact prim with no in-memory flag showing it.
        #
        # IMPORTANT, found the hard way: check the RAW xformOpOrder token
        # list, not HasAttribute(). Confirmed this session that
        # xformOpOrder's own token array can contain DANGLING entries
        # referencing an attribute that no longer exists (HasAttribute
        # returns False, but AddXformOp() still throws "already exists
        # in xformOpOrder" because it checks the order list directly,
        # not attribute presence). A HasAttribute-only check misses this
        # case entirely.
        current_order = list(xformable.GetXformOpOrderAttr().Get() or [])
        has_leftover_tokens = any(ROTATION_OP_SUFFIX in token for token in current_order)
        if has_leftover_tokens or prim.HasAttribute(_TRANSLATE_ATTR_NAME) or prim.HasAttribute(_ROTATE_ATTR_NAME):
            cleaned_order = [t for t in current_order if ROTATION_OP_SUFFIX not in t]
            xformable.GetXformOpOrderAttr().Set(cleaned_order)
            for attr_name in (_TRANSLATE_ATTR_NAME, _ROTATE_ATTR_NAME):
                if prim.HasAttribute(attr_name):
                    prim.RemoveProperty(attr_name)

        # --- record the TRUE original op order, before touching anything ---
        original_op_order = [op.GetOpName() for op in xformable.GetOrderedXformOps()]

        # --- hinge point in the PARENT's local space -----------------
        # Same computation the old sibling-controller approach used for
        # its translate op -- still valid: the pedicel's parent never
        # changes under this approach, so the parent-space hinge point
        # is identical either way.
        parent_prim = prim.GetParent()
        parent_world = omni.usd.get_world_transform_matrix(parent_prim)
        hinge_local = parent_world.GetInverse().Transform(pedicel_rig_data.hinge_point)

        # --- author the 3 ops, then reorder so they're FIRST ----------
        # Add*Op always APPENDS. Ours need to be first in xformOpOrder
        # so they wrap the ENTIRE existing stack from the outside (USD
        # composes right-to-left) -- same relationship the old sibling
        # controller had as an ancestor Xform. Reordering is NOT
        # automatic, must be done explicitly.
        translate_op = xformable.AddTranslateOp(opSuffix=f"{ROTATION_OP_SUFFIX}_pivot")
        translate_op.Set(hinge_local)
        rotate_op = xformable.AddRotateXYZOp(opSuffix=ROTATION_OP_SUFFIX)
        # was AddRotateYOp historically -- single fixed-order RotateXYZ op.
        # NEVER change this once poses have been generated with it --
        # swapping to e.g. RotateZYX silently changes what every
        # previously-recorded (x,y,z) tuple actually means.
        rotate_op.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        inv_translate_op = xformable.AddTranslateOp(opSuffix=f"{ROTATION_OP_SUFFIX}_pivot", isInverseOp=True)
        # no .Set() on the inverse -- it's paired automatically with the
        # non-inverse op of the same name; USD rejects setting it
        # directly ("set value on the paired non-inverse xformOp
        # instead"), confirmed this session.

        new_op_names = [translate_op.GetOpName(), rotate_op.GetOpName(), inv_translate_op.GetOpName()]
        # NOTE: translate_op and inv_translate_op share the same
        # underlying attribute name -- new_op_names has a deliberate
        # duplicate. Fine here (we're only using it to REORDER, not to
        # remove); reset() below dedupes before removing, where it
        # actually matters.
        all_ops_by_name = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
        new_ops = [all_ops_by_name[n] for n in new_op_names]
        original_ops = [all_ops_by_name[n] for n in original_op_order]
        xformable.SetXformOpOrder(new_ops + original_ops)

        pedicel_rig_data.controller = prim  # no separate prim anymore -- see class docstring
        pedicel_rig_data.controller_created = True
        pedicel_rig_data.current_rotation = {"x": 0.0, "y": 0.0, "z": 0.0}
        pedicel_rig_data._original_op_order = original_op_order  # TODO: promote to a real field, see class docstring

        # affected_parts: prim path never changes under this approach, so
        # this refresh is likely a no-op now rather than the fix it was
        # under the old reparenting approach -- kept anyway since it's
        # cheap and this assumption hasn't been independently verified
        # yet (flagged, not resolved, in the v17 handoff).
        pedicel_rig_data.affected_parts = list(pedicel_rig_data.prim.GetChildren())

        return pedicel_rig_data.controller

    def rotate(self, pedicel_rig_data, x_deg=0.0, y_deg=0.0, z_deg=0.0):
        if not pedicel_rig_data.controller_created:
            raise RuntimeError(
                f"{pedicel_rig_data.prim.GetPath()} has no active rotation rig. "
                f"Call create_rotation_root() first."
            )

        xformable = UsdGeom.Xformable(pedicel_rig_data.prim)
        # Match by EXACT NAME, not by type. The old code matched "the
        # RotateXYZ-type op", which was safe when the controller was a
        # fresh prim with exactly one op -- it is NOT safe now that we
        # share a prim with the pedicel's own GLTF-authored ops (which,
        # on every pedicel checked this session, include an 'orient' op
        # -- a different type, but relying on type-matching here is
        # fragile and was explicitly flagged as unsafe in the v17
        # handoff).
        for op in xformable.GetOrderedXformOps():
            if op.GetOpName() == _ROTATE_ATTR_NAME:
                op.Set(Gf.Vec3f(x_deg, y_deg, z_deg))
                pedicel_rig_data.current_rotation = {"x": x_deg, "y": y_deg, "z": z_deg}
                return

        raise RuntimeError(f"No {_ROTATE_ATTR_NAME} op found on {pedicel_rig_data.prim.GetPath()}")

    def reset(self, pedicel_rig_data):
        if not pedicel_rig_data.controller_created:
            print(f"{pedicel_rig_data.prim.GetPath()} has no active rotation rig - nothing to reset.")
            return

        prim = pedicel_rig_data.prim
        xformable = UsdGeom.Xformable(prim)
        original_op_order = pedicel_rig_data._original_op_order

        self.rotate(pedicel_rig_data, 0.0, 0.0, 0.0)

        # Dedupe before removing -- _TRANSLATE_ATTR_NAME backs BOTH the
        # non-inverse and inverse xformOpOrder entries (same underlying
        # attribute). Removing it, then later touching a stale reference
        # to the same now-deleted attribute, throws "Accessed schema on
        # invalid prim" -- hit this exact bug writing the smoke test's
        # own finish(), fixed the same way here from the start.
        for attr_name in (_TRANSLATE_ATTR_NAME, _ROTATE_ATTR_NAME):
            if prim.HasAttribute(attr_name):
                prim.RemoveProperty(attr_name)

        # Restore the TRUE original order (captured in create_rotation_root,
        # not reconstructed from what happens to be left on the stage now).
        remaining_ops_by_name = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
        restored_ops = [remaining_ops_by_name[n] for n in original_op_order if n in remaining_ops_by_name]
        xformable.SetXformOpOrder(restored_ops)

        pedicel_rig_data.affected_parts = list(pedicel_rig_data.prim.GetChildren())
        pedicel_rig_data.controller = None
        pedicel_rig_data.controller_created = False
        pedicel_rig_data.current_rotation = {"x": 0.0, "y": 0.0, "z": 0.0}
        if hasattr(pedicel_rig_data, "_original_op_order"):
            del pedicel_rig_data._original_op_order

def is_rigged(prim):
    """True if this prim's xformOpOrder already contains our tagged ops.
    Same raw-token check as the leftover-detection in create_rotation_root -
    checking HasAttribute alone misses dangling xformOpOrder tokens."""
    xformable = UsdGeom.Xformable(prim)
    order = list(xformable.GetXformOpOrderAttr().Get() or [])
    return any(ROTATION_OP_SUFFIX in token for token in order)


def get_original_op_order(prim):
    """Reconstructs the pre-rig xformOpOrder by stripping our tagged
    tokens from the current one. Only valid to call when is_rigged(prim)
    is True."""
    xformable = UsdGeom.Xformable(prim)
    order = list(xformable.GetXformOpOrderAttr().Get() or [])
    return [t for t in order if ROTATION_OP_SUFFIX not in t]


def get_current_rotation(prim):
    """Reads x/y/z off our tagged rotate op directly. Only valid to call
    when is_rigged(prim) is True."""
    xformable = UsdGeom.Xformable(prim)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpName() == _ROTATE_ATTR_NAME:
            x, y, z = op.Get()
            return {"x": x, "y": y, "z": z}
    return {"x": 0.0, "y": 0.0, "z": 0.0}




_session = {}
def get_session():
    return _session


