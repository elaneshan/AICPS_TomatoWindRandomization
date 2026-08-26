"""
manipulation_episode.py -- backend-agnostic episode loop.
"""
import json
import time




def run_episode(backend, episode_id, manip_threshold=None, metadata_log=None):
    """
    Runs one full episode against the given backend. Returns a dict
    summarizing what happened -- always returned, even on failure, so
    a batch runner can log every attempt rather than losing failed
    episodes silently.


    backend.reset() is called on EVERY exit path, not just success --
    per v14 SS3.2's stated safety reasoning (retract to a known-clear
    pose between attempts), a failed episode is exactly the case this
    matters most for, not less.
    """
    record = {"episode_id": episode_id, "status": None}


    try:
        backend.randomize_scene()


        target = backend.sample_target()
        record["target"] = target


        joint_positions = backend.solve_ik(target)
        if joint_positions is None:
            record["status"] = "ik_failed"
            return record
        record["joint_positions"] = joint_positions


        manip = backend.check_manipulability(joint_positions)
        record["manipulability"] = manip
        if manip_threshold is not None and manip["min_singular_value"] < manip_threshold:
            record["status"] = "rejected_low_manipulability"
            return record


        success, exec_info = backend.execute_trajectory(joint_positions)
        record["execute_trajectory"] = exec_info
        if not success:
            record["status"] = "trajectory_failed"
            return record


        grip_success, grip_info = backend.move_gripper(target.get("gripper_target_deg", -20.0))
        record["move_gripper"] = grip_info
        if not grip_success:
            record["status"] = "gripper_failed"
            return record


        observation = backend.capture_observation(episode_id)
        record["observation"] = observation


        record["status"] = "ok"
        return record
    finally:
        # Always retract/reset, regardless of which branch above fired --
        # reset() itself is defensive (ROSBackend.reset prints a WARNING
        # rather than raising if a sub-step fails), so this is safe even
        # on a badly-failed episode.
        backend.reset()
        _log(metadata_log, record)




def run_batch(backend, n_episodes, manip_threshold=None, metadata_path=None):
    metadata_log = []
    results = []
    for i in range(n_episodes):
        print(f"\n=== episode {i} ===")
        record = run_episode(backend, episode_id=i, manip_threshold=manip_threshold,
                              metadata_log=metadata_log)
        print(f"  status: {record['status']}")
        results.append(record)


    n_ok = sum(1 for r in results if r["status"] == "ok")
    print(f"\n=== BATCH SUMMARY: {n_ok}/{n_episodes} episodes completed successfully ===")
    status_counts = {}
    for r in results:
        status_counts[r["status"]] = status_counts.get(r["status"], 0) + 1
    for status, count in status_counts.items():
        print(f"  {status}: {count}")


    if metadata_path:
        with open(metadata_path, "w") as f:
            json.dump(results, f, indent=2, default=str)
        print(f"Wrote metadata for {len(results)} episodes to {metadata_path}")


    return results




def _log(metadata_log, record):
    if metadata_log is not None:
        metadata_log.append(dict(record))




if __name__ == "__main__":
    # Now pointed at ROSBackend (outside-Kit, two-process architecture,
    # per hand-off v16) instead of the old in-Kit SimBackend.
    from ros_backend import ROSBackend


    backend = ROSBackend()
    try:
        run_batch(
            backend,
            n_episodes=1,  # start with 1 -- per v16 SS6's own testing plan,
                            # don't jump to a full batch on first run
            manip_threshold=0.05,
            metadata_path="episode_metadata.json",
        )
    finally:
        backend.shutdown()



