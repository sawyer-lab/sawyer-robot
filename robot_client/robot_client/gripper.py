"""
GripperClient — Sawyer ClickSmart gripper control, strongly typed.
"""

from dataclasses import dataclass

from .base_client import BaseClient
from .sawyer import GripperState


@dataclass(frozen=True)
class GripperStatus:
    """Typed snapshot of the gripper returned by get_state()."""
    position:    float
    is_grasping: bool
    state:       GripperState
    device:      str

    @staticmethod
    def from_dict(d: dict) -> "GripperStatus":
        pos  = float(d.get("position", 0.0))
        grasping = bool(d.get("is_grasping", False))
        if grasping:
            state = GripperState.CLOSED
        elif pos > 0.03:
            state = GripperState.OPEN
        else:
            state = GripperState.UNKNOWN
        return GripperStatus(
            position=pos,
            is_grasping=grasping,
            state=state,
            device=str(d.get("device", "")),
        )


class GripperClient(BaseClient):
    """Controls the Sawyer ClickSmart end-effector (open / close)."""

    def open(self) -> bool:
        """Open the gripper (release any object)."""
        return self._client.gripper_open()

    def close(self) -> bool:
        """Close the gripper (grasp an object)."""
        return self._client.gripper_close()

    def get_state(self) -> GripperStatus:
        """Return the current gripper status as a typed snapshot."""
        return GripperStatus.from_dict(self._client.gripper_get_state())
