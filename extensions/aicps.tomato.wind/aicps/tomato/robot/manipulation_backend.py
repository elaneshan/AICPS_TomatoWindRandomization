
"""
manipulation_backend.py -- abstract interface both SimBackend and
RealRobotBackend implement, so manipulation_episode.py's loop never
needs to know which one it's talking to.
"""
from abc import ABC, abstractmethod


class ManipulationBackend(ABC):
    @abstractmethod
    def randomize_scene(self):
        """Sim: re-run wind rigging / cluster placement randomization.
        Real: likely a no-op, or a prompt to a human to reposition
        the physical setup -- decide per-backend, not here."""
        raise NotImplementedError

    @abstractmethod
    def sample_target(self):
        """Returns a target the rest of the pipeline can consume --
        e.g. a dict with position/orientation or a look-at point.
        Must return the SAME shape from both backends."""
        raise NotImplementedError

    @abstractmethod
    def solve_ik(self, target):
        """Returns joint_positions (list[float], 6 values) or None
        if unsolvable. None is a valid, expected return -- callers
        must handle it, not treat it as an exception."""
        raise NotImplementedError

    @abstractmethod
    def check_manipulability(self, joint_positions):
        """Returns a dict: manipulability_index, min_singular_value,
        condition_number. Shared logic in both backends -- likely
        THE SAME underlying manipulability_check.py math, since a
        real singularity is a real singularity regardless of sim vs
        hardware. Kept as its own step (not folded into solve_ik) so
        episode.py can decide what to do on a bad score without
        solve_ik needing to know about that policy."""
        raise NotImplementedError

    @abstractmethod
    def execute_trajectory(self, joint_positions):
        """Moves the arm. Returns (success: bool, info: dict)."""
        raise NotImplementedError

    @abstractmethod
    def move_gripper(self, target_deg):
        """Moves the gripper. Returns (success: bool, info: dict)."""
        raise NotImplementedError

    @abstractmethod
    def capture_observation(self, episode_id):
        """Returns a dict of whatever this backend can capture --
        images, TCP_force, pose, etc. Backends are NOT required to
        return identical keys (real has TCP_force, sim doesn't) --
        episode.py should treat this as an open dict, not a fixed
        schema, and record whatever's present.

        episode_id is passed through explicitly (rather than each
        backend tracking its own counter) so capture output naming
        always lines up exactly with the episode record it belongs to."""
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """Return to a known-safe state between episodes."""
        raise NotImplementedError

