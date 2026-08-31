"""
mailbox_listener.py -- Kit-side half of the mailbox 
"""
import json
import asyncio
import traceback


import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, Sdf


import aicps.tomato.robot.sim_internal as sim_internal


MAILBOX_PATH = "/World/EpisodeMailbox"


ASYNC_REQUEST_TYPES = {"capture"}

# Bounded wait for an in-flight capture to finish before we give up and
# tear down anyway. Should comfortably exceed a single capture's real
# duration (rt_subframes render + write) -- if this ever actually fires,
# treat it as a bug worth investigating, not a normal path.
INFLIGHT_DRAIN_TIMEOUT_SEC = 30.0


class MailboxListener:
   def __init__(self, sim=None):
       """
       Construction is now effectively two-phase:
         - __init__ (sync): builds SimInternal (which itself only does
           its own sync half -- see sim_internal.py), sets up the
           mailbox prim and update subscription, but does NOT service
           any requests yet.
         - await ready(): must be called (and awaited) once, right
           after construction, before this listener will actually
           dispatch anything. Does SimInternal's async setup
           (gripper_ready() then setup_capture()) in the correct order.


       The caller (extension.py's _start_mailbox_listener) is
       responsible for calling ready() via asyncio.ensure_future right
       after constructing this.
       """
       self.stage = omni.usd.get_context().get_stage()
       self.sim = sim if sim is not None else sim_internal.SimInternal()


       self._mailbox_prim = self._ensure_mailbox_prim()
       self._last_seen_request_id = self._read_int("request_id", default=-1)
       self._processing = False
       self._ready = False  # gate on _on_update until ready() completes

       # NEW: handle to the currently in-flight async dispatch task (if
       # any), so stop()/shutdown() can wait for it instead of racing it.
       self._inflight_task = None
       self._stop_task = None


       stream = omni.kit.app.get_app().get_update_event_stream()
       self._update_sub = stream.create_subscription_to_pop(
           self._on_update, name="episode_mailbox_listener"
       )
       print(f"mailbox_listener: watching {MAILBOX_PATH}, "
             f"starting from request_id={self._last_seen_request_id} "
             f"(not yet ready -- awaiting gripper_ready()/setup_capture())")


   async def ready(self):
       """
       Call once, right after construction. Does SimInternal's
       two-phase async setup in the correct order:
         1. gripper_ready() -- wait for gripper_sync's own startup
            window to fully pass.
         2. setup_capture() -- only then build the render_product/
            writer, avoiding the race that crashed gripper_sync's
            Articulation init this session.
       Only after both complete does _on_update start actually
       dispatching requests.
       """
       await self.sim.gripper_ready()
       await self.sim.setup_capture()
       self._ready = True
       print("mailbox_listener: ready -- now servicing requests.")


   # --- prim / attribute plumbing --------------------------------------


   def _ensure_mailbox_prim(self):
       prim = self.stage.GetPrimAtPath(MAILBOX_PATH)
       if not prim.IsValid():
           UsdGeom.Xform.Define(self.stage, MAILBOX_PATH)
           prim = self.stage.GetPrimAtPath(MAILBOX_PATH)
           print(f"mailbox_listener: created {MAILBOX_PATH}")


       prim.CreateAttribute("request_type", Sdf.ValueTypeNames.String)
       prim.CreateAttribute("request_id", Sdf.ValueTypeNames.Int)
       prim.CreateAttribute("request_payload", Sdf.ValueTypeNames.String)
       prim.CreateAttribute("response_payload", Sdf.ValueTypeNames.String)
       prim.CreateAttribute("response_id", Sdf.ValueTypeNames.Int)


       if not prim.GetAttribute("request_id").HasAuthoredValue():
           prim.GetAttribute("request_id").Set(-1)
       if not prim.GetAttribute("response_id").HasAuthoredValue():
           prim.GetAttribute("response_id").Set(-1)


       return prim


   def _read_int(self, attr_name, default=-1):
       attr = self._mailbox_prim.GetAttribute(attr_name)
       val = attr.Get()
       return default if val is None else int(val)


   def _read_str(self, attr_name, default=""):
       attr = self._mailbox_prim.GetAttribute(attr_name)
       val = attr.Get()
       return default if val is None else str(val)


   def _write_response(self, request_id, payload_dict):
       self._mailbox_prim.GetAttribute("response_payload").Set(json.dumps(payload_dict))
       self._mailbox_prim.GetAttribute("response_id").Set(request_id)


   # --- dispatch ---------------------------------------------------------


   def _on_update(self, e):
       if not self._ready:
           return
       if self._processing:
           return


       current_id = self._read_int("request_id", default=-1)
       if current_id == self._last_seen_request_id:
           return


       self._last_seen_request_id = current_id
       self._processing = True


       request_type = self._read_str("request_type")
       request_payload_raw = self._read_str("request_payload", default="{}")
       try:
           request_payload = json.loads(request_payload_raw) if request_payload_raw else {}
       except json.JSONDecodeError as ex:
           tb = traceback.format_exc()
           print(f"mailbox_listener: request_id={current_id} type={request_type} FAILED:\n{tb}")
           self._finish(current_id, {"error": f"bad request_payload JSON: {ex}", "traceback": tb})
           return


       print(f"mailbox_listener: dispatching request_id={current_id} type={request_type}")


       if request_type in ASYNC_REQUEST_TYPES:
           # NEW: store the task handle instead of discarding it, so
           # stop() can await it before tearing down SimInternal.
           self._inflight_task = asyncio.ensure_future(
               self._dispatch_async(current_id, request_type, request_payload)
           )
       else:
           self._dispatch_sync(current_id, request_type, request_payload)


   def _dispatch_sync(self, request_id, request_type, payload):
       try:
           result = self._call_sync(request_type, payload)
           self._finish(request_id, result)
       except Exception as ex:
           tb = traceback.format_exc()
           print(f"mailbox_listener: request_id={request_id} type={request_type} FAILED:\n{tb}")
           self._finish(request_id, {"error": str(ex), "traceback": tb})


   async def _dispatch_async(self, request_id, request_type, payload):
       try:
           result = await self._call_async(request_type, payload)
           self._finish(request_id, result)
       except Exception as ex:
           tb = traceback.format_exc()
           print(f"mailbox_listener: request_id={request_id} type={request_type} FAILED:\n{tb}")
           self._finish(request_id, {"error": str(ex), "traceback": tb})
       finally:
           # This task is done one way or another -- clear the handle so
           # stop() doesn't wait on a completed/stale task unnecessarily.
           self._inflight_task = None


   def _finish(self, request_id, result_dict):
       import omni.timeline
       tl = omni.timeline.get_timeline_interface()
       if not tl.is_playing():
           tl.play()
       self._write_response(request_id, result_dict)
       self._processing = False
       print(f"mailbox_listener: request_id={request_id} done")


   def _call_sync(self, request_type, payload):
       if request_type == "sample_target":
           return self.sim.sample_target()
       elif request_type == "move_gripper":
           if "target_deg" not in payload:
               raise ValueError("move_gripper request_payload must include 'target_deg'")
           success, info = self.sim.move_gripper(payload["target_deg"])
           return {"success": success, "info": info}
       elif request_type == "randomize_scene":
           result = self.sim.randomize_scene()
           return {"result": result}
       elif request_type == "sample_standoff_target":
          return self.sim.sample_standoff_target()
       else:
           raise ValueError(f"unknown request_type: {request_type!r}")


   async def _call_async(self, request_type, payload):
       if request_type == "capture":
           if "episode_id" not in payload:
               raise ValueError("capture request_payload must include 'episode_id'")
           return await self.sim.capture_observation(payload["episode_id"])
       else:
           raise ValueError(f"unknown async request_type: {request_type!r}")


   def stop(self):
       """
       Unsubscribes immediately (so no NEW request can be picked up),
       then schedules async teardown that waits for any already-in-flight
       capture to finish before shutting SimInternal down.

       Returns the asyncio.Task for that async teardown, so the caller
       (extension.py) can track/await it. This matters because
       gripper_sync.py and wrist_camera_lookat.py both hold MODULE-LEVEL
       GLOBAL state, not per-instance state -- if a second SimInternal
       gets constructed (e.g. from a Play event) while this teardown is
       still draining an in-flight capture, the two instances' calls to
       start_gripper_sync()/start_camera_lookat() collide on that shared
       global state. See extension.py's _start_after_teardown() for the
       fix on the caller side; this method just needs to expose the task
       handle for that to be possible.
       """
       if self._update_sub is not None:
           self._update_sub.unsubscribe()
           self._update_sub = None
           print("mailbox_listener: stopped.")

       self._stop_task = asyncio.ensure_future(self._stop_async())
       return self._stop_task

   async def _stop_async(self):
       if self._inflight_task is not None and not self._inflight_task.done():
           print("mailbox_listener: in-flight capture detected, waiting "
                 "for it to finish before shutting down SimInternal...")
           try:
               await asyncio.wait_for(
                   asyncio.shield(self._inflight_task),
                   timeout=INFLIGHT_DRAIN_TIMEOUT_SEC,
               )
               print("mailbox_listener: in-flight capture finished, "
                     "proceeding with shutdown.")
           except asyncio.TimeoutError:
               print(f"mailbox_listener: in-flight capture did not finish "
                     f"within {INFLIGHT_DRAIN_TIMEOUT_SEC}s -- proceeding "
                     f"with shutdown anyway. render_product/writer "
                     f"teardown may race the stuck capture; this is a bug "
                     f"if it ever actually happens, not expected behavior.")

       if self.sim is not None:
           await self.sim.shutdown()


