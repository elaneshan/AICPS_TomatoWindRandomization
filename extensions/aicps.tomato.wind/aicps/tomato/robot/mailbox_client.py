"""
mailbox_client.py -- outside-Kit half of the mailbox 

Runs as a PLAIN PYTHON SCRIPT in a normal terminal, with BOTH sourced:
    source /opt/ros/humble/setup.bash
    source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
(the second one is required here specifically because this file
constructs/parses isaac_ros2_messages/srv/{Get,Set}PrimAttribute
messages)
Talks ONLY to the two generic, already-confirmed-working services
(hand-off v16 session, manual round-trip test against
/World/EpisodeMailbox):
    isaac_ros2_messages/srv/SetPrimAttribute
    isaac_ros2_messages/srv/GetPrimAttribute
Everything about request_type / request_id / payload shape is a
convention this file and mailbox_listener.py (Kit-side) both agree on
-- there is no schema enforcement from the ROS2 service itself, so
the two files must be kept in sync by hand.

VALUE ENCODING --
Every SetPrimAttribute `value` field must be the JSON encoding of the
Python value you actually want stored 

DOUBLE-ENCODING on response_payload specifically -- mailbox_listener.py stores json.dumps(result_dict) as
response_payload's value (a String attribute). GetPrimAttribute then
JSON-encodes THAT string again on the way out (since the attribute's
own type is string). So reading a real result back needs json.loads()
TWICE 
"""
import json
import time

import rclpy
from rclpy.node import Node

from isaac_ros2_messages.srv import SetPrimAttribute, GetPrimAttribute


MAILBOX_PATH = "/World/EpisodeMailbox"  # must match mailbox_listener.py

DEFAULT_TIMEOUT_SEC = 10.0
DEFAULT_POLL_INTERVAL_SEC = 0.1
SERVICE_WAIT_TIMEOUT_SEC = 5.0
SERVICE_CALL_TIMEOUT_SEC = 30.0


class MailboxTimeoutError(RuntimeError):
    """Raised when response_id never matches the request within timeout_sec.
    Deliberately a distinct type from other RuntimeErrors this class raises
    (service-unavailable, remote-side error) so callers can tell "Kit never
    answered" apart from "Kit answered with an error" if that distinction
    ever matters."""
    pass


class MailboxClient(Node):
    """
    Does NOT call rclpy.init()/rclpy.shutdown() itself -- mirrors
    ComputeIKClient/FKClient/TrajectoryExecutorClient's own pattern
    (hand-off v16 Test C files), since ROSBackend already owns
    rclpy's process-level lifecycle and may construct several Node
    subclasses side by side. Call destroy_node() when done; let the
    owner call rclpy.shutdown() once, after all nodes are destroyed.
    """

    def __init__(self, mailbox_path=MAILBOX_PATH):
        super().__init__("mailbox_client")
        self.mailbox_path = mailbox_path

        self._set_client = self.create_client(SetPrimAttribute, "/set_prim_attribute")
        self._get_client = self.create_client(GetPrimAttribute, "/get_prim_attribute")

        if not self._set_client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            raise RuntimeError(
                "/set_prim_attribute not available after 5s -- is the "
                "ROS2ServicePrimNode in /World/ROSGraph actually initialized? "
                "(check Isaac Sim is playing and was launched from a terminal "
                "with humble_ws/install/setup.bash sourced, hand-off v16)"
            )
        if not self._get_client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            raise RuntimeError("/get_prim_attribute not available after 5s -- same check as above.")

        # Start numbering requests ABOVE whatever's already on the prim,
        # rather than always starting at 1 -- avoids colliding with a
        # request_id left over from manual testing or a prior session,
        # which mailbox_listener.py would silently ignore (it only reacts
        # to request_id CHANGING, not to any particular starting value).
        try:
            current = self._get_attribute_raw("request_id")
            self._next_request_id = int(current) + 1
        except Exception:
            self._next_request_id = 1
        self.get_logger().info(f"mailbox_client: starting at request_id={self._next_request_id}")

    # --- low-level attribute get/set, generic over String/Int ------------

    def _set_attribute_raw(self, attribute, python_value):
        """python_value: the actual value you want stored (a str or int).
        Handles the JSON-encoding convention uniformly -- see module
        docstring."""
        req = SetPrimAttribute.Request()
        req.path = self.mailbox_path
        req.attribute = attribute
        req.value = json.dumps(python_value)

        future = self._set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT_SEC)
        if not future.done():
            raise RuntimeError(f"set_prim_attribute({attribute!r}) timed out")

        resp = future.result()
        if not resp.success:
            raise RuntimeError(f"set_prim_attribute({attribute!r}) failed: {resp.message}")

    def _get_attribute_raw(self, attribute):
        """Returns the attribute's value as a real Python object (str or
        int), with the service's own JSON-encoding already undone. For
        response_payload specifically, this is still ONE level short of
        the real result dict -- see _get_json_payload()."""
        req = GetPrimAttribute.Request()
        req.path = self.mailbox_path
        req.attribute = attribute

        future = self._get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT_SEC)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"get_prim_attribute({attribute!r}) timed out")

        resp = future.result()
        if not resp.success:
            raise RuntimeError(f"get_prim_attribute({attribute!r}) failed: {resp.message}")
        return json.loads(resp.value)

    def _get_json_payload(self, attribute):
        """For String attributes whose CONTENT is itself JSON text (i.e.
        response_payload) -- see module docstring's DOUBLE-ENCODING note.
        Do the second json.loads() here, in exactly one place."""
        raw_str = self._get_attribute_raw(attribute)
        return json.loads(raw_str)

    # --- the actual request/response cycle --------------------------------

    def request(self, request_type, payload=None,
            timeout_sec=DEFAULT_TIMEOUT_SEC,
            poll_interval_sec=DEFAULT_POLL_INTERVAL_SEC):
        payload = payload if payload is not None else {}
        req_id = self._next_request_id
        self._next_request_id += 1


        self._set_attribute_raw("request_type", request_type)
        self._set_attribute_raw("request_payload", json.dumps(payload))
        self._set_attribute_raw("request_id", req_id)


        start = time.time()
        last_seen_response_id = None
        last_error = None
        while True:
            try:
                response_id = self._get_attribute_raw("response_id")
                last_seen_response_id = response_id
                if response_id == req_id:
                    break
            except Exception as e:
                last_error = e


            if time.time() - start >= timeout_sec:
                raise MailboxTimeoutError(
                    f"mailbox request {request_type!r} (id={req_id}) got no "
                    f"response within {timeout_sec}s -- is mailbox_listener.py "
                    f"running in the script editor? (still on response_id="
                    f"{last_seen_response_id}, last poll error: {last_error!r})"
                )
            time.sleep(poll_interval_sec)


        result = self._get_json_payload("response_payload")
        if isinstance(result, dict) and "error" in result:
            raise RuntimeError(
                f"mailbox request {request_type!r} (id={req_id}) failed on "
                f"the Kit side: {result['error']}"
            )
        return result





    # --- convenience wrappers, one per request_type mailbox_listener.py knows ---

    def sample_target(self):
        return self.request("sample_target", {})

    def sample_standoff_target(self):
        return self.request("sample_standoff_target", {})



    def capture_observation(self, episode_id, timeout_sec=60.0):
        return self.request("capture", {"episode_id": episode_id}, timeout_sec=timeout_sec)

    def move_gripper(self, target_deg):
        return self.request("move_gripper", {"target_deg": target_deg})

    def randomize_scene(self, timeout_sec=120.0):
        return self.request("randomize_scene", timeout_sec=timeout_sec)



if __name__ == "__main__":
    # Minimal smoke test, same shape as compute_ik_client.py's own
    # fallback -- one real sample_target request, print the result.
    rclpy.init()
    client = MailboxClient()
    try:
        target = client.sample_target()
        print("sample_target ->")
        print(json.dumps(target, indent=2))


        # calling randomize_scene
        result = client.randomize_scene()
        print("randomize_scene ->")
        print(json.dumps(result, indent=2))
    finally:
        client.destroy_node()
        rclpy.shutdown()


