"""
mailbox_listener.py -- Kit-side half of the mailbox (hand-off v16 SS4).

Runs INSIDE Isaac Sim (script editor / startup extension), never
imports rclpy. Watches a single prim's custom attributes for new
requests from the outside process, dispatches them against a
SimInternal instance, and writes JSON-encoded responses back.

The outside process talks to this ONLY via
isaac_ros2_messages/srv/SetPrimAttribute and GetPrimAttribute (both
generic: path + attribute name + JSON-string value -- confirmed shape,
hand-off v16 session). All the actual structure -- request_type,
request_id, payload encoding -- is a convention this file and the
(not-yet-written) outside mailbox_client.py both need to agree on;
there is no schema enforcement from the ROS2 service itself.

Attribute contract on the mailbox prim, all custom attributes:
    request_type     (String)  e.g. "sample_target", "capture",
                                "move_gripper", "randomize_scene"
    request_id        (Int)    increments per new request
    request_payload   (String) JSON, meaning depends on request_type
    response_payload  (String) JSON, the result
    response_id        (Int)   set to match request_id LAST, once
                                response_payload is fully written --
                                this is what tells the outside poller
                                the response is complete and not stale
                                (v16 SS4's explicit ordering requirement).

USAGE (script editor, scene already has rig + robot + Play pressed):
    import mailbox_listener
    listener = mailbox_listener.MailboxListener()
    # leave running -- it services requests every frame from here on
"""
import json
import asyncio
import traceback

import omni.kit.app
import omni.usd
from pxr import Usd, UsdGeom, Sdf

import aicps.tomato.robot.sim_internal as sim_internal


MAILBOX_PATH = "/World/EpisodeMailbox"

# request_type -> whether the corresponding SimInternal call is a
# coroutine (needs await) or a plain synchronous call. Kept explicit
# here rather than inferred via inspect.iscoroutinefunction, so a
# future request type can't silently get the wrong dispatch path.
ASYNC_REQUEST_TYPES = {"capture"}


class MailboxListener:
    def __init__(self, sim=None):
        """
        sim: an existing SimInternal instance, or None to construct a
        fresh one. Accepting an existing instance matters if
        something else in the session already built one (e.g. during
        interactive testing, per the previous sim_internal.py smoke
        test) -- constructing a SECOND SimInternal would build a
        second rig/checker/controller_tool against the same stage,
        which is wasteful at best and has not been tested for
        correctness at worst. Prefer passing the existing one.
        """
        self.stage = omni.usd.get_context().get_stage()
        self.sim = sim if sim is not None else sim_internal.SimInternal()

        self._mailbox_prim = self._ensure_mailbox_prim()
        self._last_seen_request_id = self._read_int("request_id", default=-1)
        self._processing = False

        stream = omni.kit.app.get_app().get_update_event_stream()
        self._update_sub = stream.create_subscription_to_pop(
            self._on_update, name="episode_mailbox_listener"
        )
        print(f"mailbox_listener: watching {MAILBOX_PATH}, "
              f"starting from request_id={self._last_seen_request_id}")

    # --- prim / attribute plumbing --------------------------------------

    def _ensure_mailbox_prim(self):
        prim = self.stage.GetPrimAtPath(MAILBOX_PATH)
        if not prim.IsValid():
            UsdGeom.Xform.Define(self.stage, MAILBOX_PATH)
            prim = self.stage.GetPrimAtPath(MAILBOX_PATH)
            print(f"mailbox_listener: created {MAILBOX_PATH}")

        # CreateAttribute is idempotent -- safe to call every time even
        # if the prim (and its attributes) already existed from a prior
        # session, per USD's own semantics.
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
        # payload_dict written FIRST, response_id LAST -- an outside
        # poller reading response_id == its own request_id is only a
        # valid signal if response_payload is guaranteed already
        # written by that point. Order matters here, do not swap it.
        self._mailbox_prim.GetAttribute("response_payload").Set(json.dumps(payload_dict))
        self._mailbox_prim.GetAttribute("response_id").Set(request_id)

    # --- dispatch ---------------------------------------------------------

    def _on_update(self, e):
        if self._processing:
            return

        current_id = self._read_int("request_id", default=-1)
        if current_id == self._last_seen_request_id:
            return

        # Claim this request immediately, before any await -- prevents
        # re-triggering on the same request_id across subsequent frames
        # while an async dispatch (e.g. "capture") is still in flight.
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
            asyncio.ensure_future(self._dispatch_async(current_id, request_type, request_payload))
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
        if self._update_sub is not None:
            self._update_sub.unsubscribe()
            self._update_sub = None
            print("mailbox_listener: stopped.")

