import asyncio
import omni.ext
import omni.timeline
import carb


TEARDOWN_DEBOUNCE_SEC = 2.0  # gripper_sync.start_gripper_sync() intentionally
                              # does stop() -> patch schema -> play() every time
                              # it runs (v13's lesson: schema edits must happen
                              # while stopped). That fires real global timeline
                              # STOP/PLAY events as a side effect. Debounce so
                              # that self-inflicted pair isn't mistaken for a
                              # real user Stop -- only tear down if the timeline
                              # is STILL stopped after this window.


class TomatoWindExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        carb.log_warn("================================")
        carb.log_warn("Hello from AICPS Wind Extension")
        carb.log_warn("================================")

        self._listener = None
        self._pending_teardown_task = None

        timeline = omni.timeline.get_timeline_interface()
        self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline_event, name="aicps_tomato_wind_timeline_listener"
        )

    def _on_timeline_event(self, event):
        import omni.timeline as tl_module

        if event.type == int(tl_module.TimelineEventType.PLAY):
            self._cancel_pending_teardown()
            if self._listener is None:
                carb.log_warn("aicps.tomato.wind: Play detected, starting mailbox listener...")
                self._start_mailbox_listener()
            else:
                carb.log_warn("aicps.tomato.wind: Play detected but listener already running "
                               "(likely gripper_sync's own internal stop/play cycle) -- not rebuilding.")

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
        except Exception as ex:
            carb.log_error(f"aicps.tomato.wind: failed to start mailbox listener: {ex!r}")
            self._listener = None

    def _teardown_mailbox_listener(self):
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception as ex:
                carb.log_warn(f"aicps.tomato.wind: error stopping mailbox listener: {ex!r}")
            self._listener = None

    def on_shutdown(self):
        carb.log_warn("AICPS Wind Extension shutdown")
        self._cancel_pending_teardown()
        self._teardown_mailbox_listener()
        if self._timeline_sub is not None:
            self._timeline_sub.unsubscribe()
            self._timeline_sub = None

