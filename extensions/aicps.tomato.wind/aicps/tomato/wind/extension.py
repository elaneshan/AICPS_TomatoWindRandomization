import asyncio
import omni.ext
import omni.timeline
import carb




TEARDOWN_DEBOUNCE_SEC = 2.0




class TomatoWindExtension(omni.ext.IExt):


   def on_startup(self, ext_id):
       carb.log_warn("================================")
       carb.log_warn("Hello from AICPS Wind Extension")
       carb.log_warn("================================")


       self._listener = None
       self._pending_teardown_task = None

       self._teardown_task = None


       timeline = omni.timeline.get_timeline_interface()
       self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
           self._on_timeline_event, name="aicps_tomato_wind_timeline_listener"
       )


       # If the extension reloads (e.g. hot-reload from a file save) while
       # the timeline is ALREADY playing, no new PLAY event will ever fire
       # -- the timeline's state didn't change, so there's nothing for
       # _on_timeline_event to react to. Check directly here and start
       # immediately in that case, instead of waiting for an event that
       # will never come.
       if timeline.is_playing():
           carb.log_warn("aicps.tomato.wind: extension (re)started while timeline "
                          "already playing -- starting mailbox listener directly.")
           self._start_mailbox_listener()




   def _on_timeline_event(self, event):
       import omni.timeline as tl_module


       if event.type == int(tl_module.TimelineEventType.PLAY):
           self._cancel_pending_teardown()
           if self._listener is None:
               if self._teardown_task is not None and not self._teardown_task.done():
                   # A previous listener's teardown is still draining an
                   # in-flight capture (can take up to 30s). Building a new
                   # SimInternal right now would collide with it via
                   # gripper_sync/wrist_camera_lookat's shared global
                   # state. Wait for that teardown to actually finish
                   # first, then start -- rather than racing it.
                   carb.log_warn("aicps.tomato.wind: Play detected, but previous "
                                  "listener teardown is still in progress (draining "
                                  "an in-flight capture) -- deferring new listener "
                                  "startup until teardown completes, to avoid "
                                  "colliding with it on gripper_sync/wrist_camera_lookat's "
                                  "shared global state...")
                   asyncio.ensure_future(self._start_after_teardown(self._teardown_task))
               else:
                   carb.log_warn("aicps.tomato.wind: Play detected, starting mailbox listener...")
                   self._start_mailbox_listener()
           else:
               carb.log_warn("aicps.tomato.wind: Play detected but listener already running "
                              "(likely gripper_sync's own internal stop/play cycle) -- not rebuilding.")
           print(f"aicps.tomato.wind: timeline event {tl_module.TimelineEventType(event.type).name} received, "
                       f"listener running={self._listener is not None}, "
                       f"pending teardown={self._pending_teardown_task is not None}, "
                       f"prior teardown draining={self._teardown_task is not None and not self._teardown_task.done()}")
       elif event.type == int(tl_module.TimelineEventType.STOP):
           if self._listener is not None:
               carb.log_warn(f"aicps.tomato.wind: Stop detected, scheduling teardown in "
                              f"{TEARDOWN_DEBOUNCE_SEC}s unless Play fires first "
                              f"(guards against gripper_sync's internal stop/play cycle)...")
               self._schedule_teardown()


   def _schedule_teardown(self):
       self._cancel_pending_teardown()
       self._pending_teardown_task = asyncio.ensure_future(self._debounced_teardown())


   async def _debounced_teardown(self):
       await asyncio.sleep(TEARDOWN_DEBOUNCE_SEC)
       timeline = omni.timeline.get_timeline_interface()
       if not timeline.is_playing():
           carb.log_warn("aicps.tomato.wind: still stopped after debounce -- "
                          "real Stop, tearing down mailbox listener.")
           self._teardown_mailbox_listener()
       else:
           carb.log_warn("aicps.tomato.wind: timeline resumed before debounce elapsed -- "
                          "self-inflicted stop/play, not tearing down.")
       self._pending_teardown_task = None


   def _cancel_pending_teardown(self):
       if self._pending_teardown_task is not None and not self._pending_teardown_task.done():
           self._pending_teardown_task.cancel()
       self._pending_teardown_task = None


   def _start_mailbox_listener(self):
       try:
           import aicps.tomato.robot.mailbox_listener as mailbox_listener_module
           self._listener = mailbox_listener_module.MailboxListener()


           asyncio.ensure_future(self._listener.ready())
       except Exception as ex:
           carb.log_error(f"aicps.tomato.wind: failed to start mailbox listener: {ex!r}")
           self._listener = None


   async def _start_after_teardown(self, teardown_task):
       """
       Waits for a previous listener's in-flight teardown to fully finish
       (including its up-to-30s drain of any in-flight capture, per
       mailbox_listener.py's stop()/_stop_async()), THEN starts a fresh
       listener -- avoiding the gripper_sync/wrist_camera_lookat global-
       state collision that results from building a new SimInternal while
       an old one is still tearing down.
       """
       try:
           await teardown_task
       except Exception as ex:
           carb.log_warn(f"aicps.tomato.wind: previous teardown raised while "
                          f"we were waiting on it: {ex!r} -- starting new "
                          f"listener anyway, since the teardown attempt is "
                          f"done either way.")

       timeline = omni.timeline.get_timeline_interface()
       if self._listener is not None:
           # Something else (e.g. another Play/reload racing this one)
           # already started a listener while we were waiting -- don't
           # start a second one on top of it.
           carb.log_warn("aicps.tomato.wind: a listener was started by "
                          "something else while we waited for the previous "
                          "teardown -- not starting a duplicate.")
           return
       if not timeline.is_playing():
           # Timeline stopped again while we were waiting for the old
           # teardown to finish -- don't start a listener just to have it
           # immediately scheduled for teardown again.
           carb.log_warn("aicps.tomato.wind: timeline is no longer playing "
                          "now that the previous teardown finished -- not "
                          "starting a new listener.")
           return

       carb.log_warn("aicps.tomato.wind: previous listener teardown finished -- "
                      "starting mailbox listener now.")
       self._start_mailbox_listener()




   def _teardown_mailbox_listener(self):
       if self._listener is not None:
           try:
               # stop() now returns the async teardown Task -- keep a
               # handle to it so a Play arriving before it finishes can
               # wait on it instead of racing it (see _on_timeline_event).
               self._teardown_task = self._listener.stop()
           except Exception as ex:
               carb.log_warn(f"aicps.tomato.wind: error stopping mailbox listener: {ex!r}")
               self._teardown_task = None
           self._listener = None


   def on_shutdown(self):
       carb.log_warn("AICPS Wind Extension shutdown")
       self._cancel_pending_teardown()
       self._teardown_mailbox_listener()
       if self._timeline_sub is not None:
           self._timeline_sub.unsubscribe()
           self._timeline_sub = None


