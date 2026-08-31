"""
episode_capture.py -- grabs ONE frame from EyeInHand_Camera for a single
manipulation episode, into its own per-episode output subfolder.

"""
import os
import shutil
import omni.replicator.core as rep
import omni.timeline




EYE_IN_HAND_CAMERA_PATH = (
   "/World/cr3/Geometry/world/dummy_link/base_link/Link1/Link2/Link3/"
   "Link4/Link5/Link6/Gripper/Geometry/gripper_base_link/EyeInHand_Camera"
)




def _check_render_product_alive(render_product, where):
   """Raises a clear, diagnosable error if render_product (or its
   hydra_texture) has been torn down out from under us -- e.g. by a
   concurrent SimInternal.shutdown() destroying it while we were
   suspended at an `await`. Should never fire given mailbox_listener.py's
   drain-before-shutdown sequencing; if it does, that sequencing has a
   new hole and needs re-checking, not a silent retry here."""
   if render_product is None or getattr(render_product, "hydra_texture", None) is None:
       raise RuntimeError(
           f"episode_capture: render_product was destroyed mid-capture "
           f"(detected {where}) -- this means something tore down "
           f"SimInternal's capture pipeline while a capture was still "
           f"in flight. Check mailbox_listener.py's stop()/_stop_async() "
           f"drain-before-shutdown sequencing rather than treating this "
           f"as expected."
       )


async def capture_episode_frame(render_product, writer, output_dir, episode_id, frame_number, rt_subframes=32):
    _check_render_product_alive(render_product, "at start of capture_episode_frame")


    render_product.hydra_texture.set_updates_enabled(True)
    await rep.orchestrator.step_async(rt_subframes=rt_subframes, pause_timeline=False)
    _check_render_product_alive(render_product, "after step_async")
    render_product.hydra_texture.set_updates_enabled(False)


    tl = omni.timeline.get_timeline_interface()
    if not tl.is_playing():
        tl.play()


    episode_dir = os.path.join(output_dir, f"episode_{episode_id:04d}")
    os.makedirs(episode_dir, exist_ok=True)


    moved = []
    for fname in os.listdir(output_dir):
        if fname.startswith(f"rgb_{frame_number:04d}") or fname.startswith(f"distance_to_camera_{frame_number:04d}"):
            shutil.move(os.path.join(output_dir, fname), os.path.join(episode_dir, fname))
            moved.append(fname)


    return {"captured": True, "episode_dir": episode_dir, "files": moved}





