"""
episode_capture.py -- grabs ONE frame from EyeInHand_Camera for a single
manipulation episode.
"""
import os
import time
import omni.replicator.core as rep
import omni.timeline

EYE_IN_HAND_CAMERA_PATH = (
    "/World/cr3/Geometry/world/dummy_link/base_link/Link1/Link2/Link3/"
    "Link4/Link5/Link6/Gripper/Geometry/gripper_base_link/EyeInHand_Camera"
)


async def capture_episode_frame(output_dir, episode_id, resolution=(1280, 720),
                                 capture_depth=True, rt_subframes=32):
    episode_dir = os.path.join(output_dir, f"episode_{episode_id:04d}")
    os.makedirs(episode_dir, exist_ok=True)

    render_product = rep.create.render_product(EYE_IN_HAND_CAMERA_PATH, resolution)
    writer = rep.writers.get("BasicWriter")
    writer.initialize(output_dir=episode_dir, rgb=True, distance_to_camera=capture_depth)
    writer.attach([render_product])

    await rep.orchestrator.step_async(rt_subframes=rt_subframes, pause_timeline=False)
    await rep.orchestrator.wait_until_complete_async()

    writer.detach()
    render_product.destroy()

    # wait_until_complete_async() stops the timeline as a side effect of how
    # Replicator guarantees a deterministic capture -- but the ROS2 bridge
    # (OmniGraph) only ticks while playing, so leaving it stopped here
    # silently kills every mailbox response after this point. Resume it
    # before returning.
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_playing():
        tl.play()

    return {"output_dir": episode_dir}



