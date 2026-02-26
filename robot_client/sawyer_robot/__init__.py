"""
sawyer_robot — Sawyer robot control library.

Entry point::

    from sawyer_robot import SawyerRobot

    with SawyerRobot(host='localhost') as robot:
        joints = robot.arm.get_joints()
        joints.j0 += 0.05
        robot.arm.move(joints)
        robot.gripper.open()

Geometry primitives: Position, Quaternion, Euler, Pose, JointAngles
Sawyer enums:        Joint, Camera, Limb, GripperState
Status types:        GripperStatus
"""

from .sawyer_robot import SawyerRobot
# Re-export geometry from common package for backward compatibility
from sawyer_common.geometry import Position, Quaternion, Euler, Pose, JointAngles
from .sawyer import Joint, Camera, Limb, GripperState
from .components.gripper import GripperStatus

__version__ = "2.0.0"
__all__ = [
    # entry point
    "SawyerRobot",
    # geometry
    "Position",
    "Quaternion",
    "Euler",
    "Pose",
    "JointAngles",
    # enums
    "Joint",
    "Camera",
    "Limb",
    "GripperState",
    # status types
    "GripperStatus",
]
