"""
Gripper — Sawyer ClickSmart end-effector component.
"""

from dataclasses import dataclass

from ..sawyer import GripperState


@dataclass(frozen=True)
class GripperStatus:
    """Typed snapshot of the gripper state."""
    position:    float
    is_grasping: bool
    state:       GripperState
    device:      str

    @staticmethod
    def from_dict(d: dict) -> "GripperStatus":
        pos      = float(d.get('position', 0.0))
        grasping = bool(d.get('is_grasping', False))
        if grasping:
            st = GripperState.CLOSED
        elif pos > 0.03:
            st = GripperState.OPEN
        else:
            st = GripperState.UNKNOWN
        return GripperStatus(
            position=pos,
            is_grasping=grasping,
            state=st,
            device=str(d.get('device', '')),
        )


class Gripper:
    """Controls the ClickSmart end-effector. Obtained via ``robot.gripper``."""

    def __init__(self, client) -> None:
        self._client = client

    def open(self) -> bool:
        """Open the gripper (release any object)."""
        return self._client.gripper_open()

    def release_async(self) -> bool:
        """Release gripper asynchronously (non-blocking)."""
        return self._client.gripper_release_async()

    def close(self) -> bool:
        """Close the gripper (grasp an object)."""
        return self._client.gripper_close()

    @property
    def state(self) -> GripperStatus:
        """Live gripper status snapshot."""
        return GripperStatus.from_dict(self._client.gripper_get_state())
