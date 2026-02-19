"""
RobotClient — Sawyer arm control, strongly typed.
"""

from typing import Optional
import numpy as np

from .base_client import BaseClient
from .geometry import JointAngles, Pose, Position, Quaternion
from .sawyer import Limb


class RobotClient(BaseClient):
    """Controls the Sawyer right arm (joint positions, Cartesian pose, trajectories)."""

    # ── joint control ─────────────────────────────────────────────────────────

    def move_to_joints(self, angles: JointAngles, timeout: float = 30.0) -> bool:
        """Move the arm to the given joint configuration."""
        return self._client.move_to_joints(angles.to_list(), timeout)

    def get_joint_angles(self) -> JointAngles:
        """Return current joint angles."""
        return JointAngles.from_list(self._client.get_joint_angles())

    def get_joint_velocities(self) -> JointAngles:
        """Return current joint velocities (rad/s)."""
        return JointAngles.from_list(self._client.get_joint_velocities())

    # ── Cartesian control ─────────────────────────────────────────────────────

    def get_endpoint_pose(self) -> Optional[Pose]:
        """Return current end-effector pose, or None if unavailable."""
        raw = self._client.get_endpoint_pose()
        return Pose.from_dict(raw) if raw else None

    def get_endpoint_velocity(self) -> Optional[dict]:
        """Return current end-effector linear and angular velocity."""
        return self._client.get_endpoint_velocity()

    # ── trajectories ──────────────────────────────────────────────────────────

    def execute_trajectory(self, waypoints: list[dict], rate_hz: float = 100.0) -> bool:
        """
        Execute a pre-computed trajectory at the given rate.

        Each waypoint is a dict with keys 'position', 'velocity', 'acceleration',
        each mapping to a list of 7 floats.
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
            release_index: Trajectory step at which to open the gripper.
                           Pass None to skip gripper release.
        """
        return self._client.execute_toss_trajectory(Q, Qd, Qdd, release_index)

    # ── robot info ────────────────────────────────────────────────────────────

    def get_state(self) -> dict:
        """Return complete robot state dict."""
        return self._client.get_robot_state()

    def get_joint_names(self, limb: Limb = Limb.RIGHT) -> list[str]:
        """Return the ROS joint names for the given limb."""
        return self._client.params_joint_names(limb.value)

    # ── gripper convenience ───────────────────────────────────────────────────

    def gripper_open(self) -> bool:
        """Open the gripper."""
        return self._client.gripper_open()

    def gripper_close(self) -> bool:
        """Close the gripper."""
        return self._client.gripper_close()

    def gripper_get_state(self) -> dict:
        """Return current gripper state."""
        return self._client.gripper_get_state()
