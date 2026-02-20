"""
Arm — Sawyer right-arm control component.
"""

from typing import Optional, List, Dict
import numpy as np
from dataclasses import dataclass, field

from ..geometry import JointAngles, Pose


@dataclass
class InteractionOptions:
    """
    Options for force and stiffness (impedance) control.
    """
    active: bool = True
    
    # Modes for each of 6 Cartesian axes (1=IMPEDANCE, 2=FORCE, 3=IMPEDANCE_LIMIT, 4=FORCE_LIMIT)
    # Default is all IMPEDANCE_MODE
    interaction_control_mode: List[int] = field(default_factory=lambda: [1]*6)
    
    # Stiffness (N/m or Nm/rad)
    K_impedance: List[float] = field(default_factory=lambda: [1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0])
    
    # Damping (Ns/m or Nms/rad)
    D_impedance: List[float] = field(default_factory=lambda: [8.0, 8.0, 8.0, 2.0, 2.0, 2.0])
    
    # Joint Nullspace stiffness (Nm/rad)
    K_nullspace: List[float] = field(default_factory=lambda: [10.0]*7)
    
    # Force/Torque command (N or Nm)
    force_command: List[float] = field(default_factory=lambda: [0.0]*6)
    
    # Whether to use max impedance for certain directions
    max_impedance: List[bool] = field(default_factory=lambda: [False]*6)
    
    # Reference frame
    in_endpoint_frame: bool = False
    interaction_frame: Pose = field(default_factory=lambda: Pose.identity())
    
    disable_damping_in_force_control: bool = False
    disable_reference_resetting: bool = False
    rotations_for_constrained_zeroG: bool = False

    def to_dict(self) -> Dict:
        return {
            'active': self.active,
            'interaction_control_mode': self.interaction_control_mode,
            'K_impedance': self.K_impedance,
            'D_impedance': self.D_impedance,
            'K_nullspace': self.K_nullspace,
            'force_command': self.force_command,
            'max_impedance': self.max_impedance,
            'in_endpoint_frame': self.in_endpoint_frame,
            'interaction_frame': self.interaction_frame.to_dict(),
            'disable_damping_in_force_control': self.disable_damping_in_force_control,
            'disable_reference_resetting': self.disable_reference_resetting,
            'rotations_for_constrained_zeroG': self.rotations_for_constrained_zeroG
        }


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

    def get_efforts(self) -> JointAngles:
        """Return current joint efforts (Nm)."""
        return JointAngles.from_list(self._client.get_joint_efforts())

    # ── direct control modes ──────────────────────────────────────────────────

    def set_positions(self, joints: JointAngles) -> bool:
        """Command raw joint positions (POSITION_MODE)."""
        return self._client.set_joint_positions(joints.to_list())

    def set_velocities(self, velocities: JointAngles) -> bool:
        """Command joint velocities (VELOCITY_MODE)."""
        return self._client.set_joint_velocities(velocities.to_list())

    def set_torques(self, torques: JointAngles) -> bool:
        """Command joint torques (TORQUE_MODE)."""
        return self._client.set_joint_torques(torques.to_list())

    def set_trajectory(self, joints: JointAngles, velocities: JointAngles, accelerations: JointAngles) -> bool:
        """Command joint trajectory (TRAJECTORY_MODE)."""
        # Note: This is a single-step command. For a full trajectory use stream_trajectory.
        waypoints = [{
            'position': joints.to_list(),
            'velocity': velocities.to_list(),
            'acceleration': accelerations.to_list()
        }]
        return self._client.execute_trajectory(waypoints)

    def set_interaction_options(self, options: InteractionOptions) -> bool:
        """
        Set force and stiffness (impedance) control options.
        
        Args:
            options: InteractionOptions object.
        """
        return self._client.set_interaction_options(options.to_dict())

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

    def stream_trajectory(
        self,
        Q:   np.ndarray,
        Qd:  np.ndarray,
        Qdd: np.ndarray,
        release_index: Optional[int] = None,
    ) -> bool:
        """
        Stream trajectory at 100 Hz with async gripper release.

        The server runs the loop and handles the timing and async gripper firing.

        Args:
            Q:             (N, 7) joint positions in radians.
            Qd:            (N, 7) joint velocities in rad/s.
            Qdd:           (N, 7) joint accelerations in rad/s².
            release_index: Step at which to open the gripper.
        """
        return self._client.execute_stream_trajectory(Q, Qd, Qdd, release_index)
