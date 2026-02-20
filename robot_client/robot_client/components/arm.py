"""
Arm — Sawyer right-arm control component.
"""

from typing import Optional
import numpy as np

from ..geometry import JointAngles, Pose


class Arm:
    """Controls the Sawyer right arm. Obtained via ``robot.arm``."""

    def __init__(self, client) -> None:
        self._client = client

    # ── joint space ───────────────────────────────────────────────────────────

    def get_joints(self) -> JointAngles:
        """Return current joint angles."""
        return JointAngles.from_list(self._client.get_joint_angles())

    def move(self, joints: JointAngles, timeout: float = 30.0) -> bool:
        """Move to the given joint configuration."""
        return self._client.move_to_joints(joints.to_list(), timeout)

    def get_velocities(self) -> JointAngles:
        """Return current joint velocities (rad/s)."""
        return JointAngles.from_list(self._client.get_joint_velocities())

    # ── Cartesian space ───────────────────────────────────────────────────────

    def get_pose(self) -> Optional[Pose]:
        """Return current end-effector pose, or None if unavailable."""
        raw = self._client.get_endpoint_pose()
        return Pose.from_dict(raw) if raw else None

    def get_velocity(self) -> Optional[dict]:
        """Return end-effector linear and angular velocity dict."""
        return self._client.get_endpoint_velocity()

    # ── state ─────────────────────────────────────────────────────────────────

    def get_state(self) -> dict:
        """Return full robot state dict."""
        return self._client.get_robot_state()

    # ── trajectories ──────────────────────────────────────────────────────────

    def execute_trajectory(
        self,
        waypoints: list,
        rate_hz: float = 100.0,
    ) -> bool:
        """
        Execute a pre-computed trajectory.

        Each waypoint is a dict with 'position', 'velocity', 'acceleration'
        keys, each a list of 7 floats.
        """
        return self._client.execute_trajectory(waypoints, rate_hz)

    def execute_toss_trajectory(
        self,
        Q:   np.ndarray,
        Qd:  np.ndarray,
        Qdd: np.ndarray,
        release_index: Optional[int] = None,
    ) -> bool:
        """
        Execute a toss trajectory at 100 Hz with async gripper release.

        Args:
            Q:             (N, 7) joint positions in radians.
            Qd:            (N, 7) joint velocities in rad/s.
            Qdd:           (N, 7) joint accelerations in rad/s².
            release_index: Step at which to open the gripper.
        """
        return self._client.execute_toss_trajectory(Q, Qd, Qdd, release_index)
