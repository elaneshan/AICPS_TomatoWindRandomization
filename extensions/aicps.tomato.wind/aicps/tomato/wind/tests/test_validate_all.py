
import omni.usd
from pxr import Gf
import aicps.tomato.wind.rig as rig_module
import aicps.tomato.wind.transform as transform_module


def _world_pos(prim):
    """World-space translation component of a prim, for before/after comparisons."""
    matrix = omni.usd.get_world_transform_matrix(prim)
    return matrix.ExtractTranslation()


def _authored_pivot_world_estimate(pedicel_prim):
    """
    Best-effort comparison point: if the imported asset already has its own
    xformOp:translate:pivot (like Pedicel_01 does), report it alongside our
    computed hinge so we can eyeball how close our geometric method lands.
    This is approximate - it doesn't fully unwind the op stack - just a sanity signal.
    """
    attr = pedicel_prim.GetAttribute("xformOp:translate:pivot")
    if not attr or not attr.IsValid(): #so if it doesn't have a pivot at all (this should not run theoretically)
        return None

    pivot_local = attr.Get()
    if pivot_local is None:
        return None

    parent_world = omni.usd.get_world_transform_matrix(pedicel_prim.GetParent())
    return parent_world.Transform(Gf.Vec3d(pivot_local))

def run_fidelity_check(angle=3.0, epsilon=1e-4):
    """
    Non-interactive pass over all discovered pedicels:
      - records world position before touching anything
      - creates controller, rotates, resets
      - confirms world position after reset matches the original (within epsilon)
      - reports computed hinge vs. any authored pivot for comparison
    Does NOT verify visual correctness of the rotation itself - see step() for that.
    """
    stage = omni.usd.get_context().get_stage()

    rig = rig_module.PlantRig(stage)
    rig.build()

    print(f"\n===== Validating {len(rig.pedicels)} pedicels (test angle={angle}) =====")

    results = []

    for pedicel in rig.pedicels:
        name = pedicel.prim.GetName()
        controller_tool = transform_module.TransformController(stage)

        before_pos = _world_pos(pedicel.prim)
        authored_pivot = _authored_pivot_world_estimate(pedicel.prim)

        try:
            controller = controller_tool.create_rotation_root(pedicel)
            controller_tool.rotate(pedicel, angle)
            controller_tool.reset(pedicel)

            after_pos = _world_pos(pedicel.prim)
            delta = (after_pos - before_pos).GetLength()
            passed = delta < epsilon

        except Exception as e:
            delta = None
            passed = False
            print(f"  [{name}] EXCEPTION: {e}")

        status = "PASS" if passed else "FAIL"
        print(f"  [{name}] reset_fidelity={status}  delta={delta}")
        print(f"      computed_hinge = {pedicel.hinge_point}")
        if authored_pivot is not None:
            hinge_vs_authored = (Gf.Vec3d(pedicel.hinge_point) - authored_pivot).GetLength()
            print(f"      authored_pivot = {authored_pivot}  (diff from computed: {hinge_vs_authored:.4f})")
        else:
            print(f"      authored_pivot = none found on this prim")

        results.append({
            "name": name,
            "reset_ok": passed,
            "delta": delta,
            "hinge": pedicel.hinge_point,
            "authored_pivot": authored_pivot,
        })

    print("\n===== Summary =====")
    failed = [r["name"] for r in results if not r["reset_ok"]]
    if failed:
        print(f"FAILED reset fidelity: {failed}")
    else:
        print("All pedicels reset cleanly.")
    print("====================\n")

    return results


# ---- Interactive step-through for visual inspection ----


# had to change this from lookign through the registry array based on index
# when a pedicel is rotated and reset, the order of the registry array changes and it was causing the fucntion to call on pedicels
# that were already rotated and reset, so now you have to call the pedicel by name to work around that 
def step(pedicel_name, angle=3.0):
    """
    Rotates ONE pedicel and leaves it rotated so you can look at it in the
    viewport. Run finish_step() afterward to reset before moving to the next.
    """
    stage = omni.usd.get_context().get_stage()

    rig = rig_module.PlantRig(stage)
    rig.build()

    match = next((p for p in rig.pedicels if p.prim.GetName() == pedicel_name), None)
    if match is None:
        available = [p.prim.GetName() for p in rig.pedicels]
        raise RuntimeError(f"Pedicel '{pedicel_name}' not found. Available: {available}")
    
    controller_tool = transform_module.TransformController(stage)

    print(f"[{match.prim.GetName()}] hinge={match.hinge_point}")
    controller = controller_tool.create_rotation_root(match)
    controller_tool.rotate(match, angle)
    print(f"Rotated {angle} deg. Inspect viewport, then call finish_step().")

    session = transform_module.get_session()
    session["controller_tool"] = controller_tool
    session["pedicel"] = match


def finish_step():
    session = transform_module.get_session()
    if "controller_tool" not in session:
        print("Nothing active — run step() first.")
        return

    session["controller_tool"].reset(session["pedicel"])
    session.clear()
    print("Reset complete.")

