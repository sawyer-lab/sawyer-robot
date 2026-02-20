"""
SawyerRobot — single entry point for all Sawyer robot control.

Usage::

    from robot_client import SawyerRobot

    robot = SawyerRobot(host='localhost')

    joints = robot.arm.get_joints()
    joints.j0 += 0.05
    robot.arm.move(joints)

    robot.gripper.open()
    frame = robot.head_camera.get_image()
"""

from .protocols.zmq import ZMQClient
from .sawyer import Camera
from .components.arm import Arm
from .components.gripper import Gripper
from .components.camera import BoundCamera
from .components.head import Head
from .components.lights import Lights


class SawyerRobot:
    """
    Facade for the Sawyer robot.

    All sub-systems are exposed as properties and share a single
    ZMQ connection to the robot container.
    """

    def __init__(self, host: str = 'localhost', port: int = 5555) -> None:
        self._client = ZMQClient(host, port)
        self._arm:         Arm          | None = None
        self._gripper:     Gripper      | None = None
        self._head_camera: BoundCamera  | None = None
        self._hand_camera: BoundCamera  | None = None
        self._head:        Head         | None = None
        self._lights:      Lights       | None = None

    # ── sub-systems ───────────────────────────────────────────────────────────

    @property
    def arm(self) -> Arm:
        """Right arm — joint movement and Cartesian control."""
        if self._arm is None:
            self._arm = Arm(self._client)
        return self._arm

    @property
    def gripper(self) -> Gripper:
        """ClickSmart end-effector — open / close / state."""
        if self._gripper is None:
            self._gripper = Gripper(self._client)
        return self._gripper

    @property
    def head_camera(self) -> BoundCamera:
        """Head-mounted camera."""
        if self._head_camera is None:
            self._head_camera = BoundCamera(self._client, Camera.HEAD)
        return self._head_camera

    @property
    def hand_camera(self) -> BoundCamera:
        """Wrist / end-effector camera."""
        if self._hand_camera is None:
            self._hand_camera = BoundCamera(self._client, Camera.HAND)
        return self._hand_camera

    @property
    def head(self) -> Head:
        """Head pan and display."""
        if self._head is None:
            self._head = Head(self._client)
        return self._head

    @property
    def lights(self) -> Lights:
        """Robot lights."""
        if self._lights is None:
            self._lights = Lights(self._client)
        return self._lights

    # ── connection ────────────────────────────────────────────────────────────

    def ping(self) -> float:
        """Round-trip latency to the robot container in milliseconds."""
        return self._client.ping()

    def close(self) -> None:
        """Close the ZMQ connection."""
        if hasattr(self._client, 'close'):
            self._client.close()

    def __enter__(self) -> "SawyerRobot":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"SawyerRobot(host={self._client.host!r})"
