"""
sawyer.py — Sawyer-specific constants and enumerations.

Defines all named entities for the Sawyer robot so callers never
pass raw strings or magic numbers.
"""

from enum import Enum


class Joint(str, Enum):
    """The seven joints of the Sawyer right arm, in proximal→distal order."""
    J0 = 'right_j0'
    J1 = 'right_j1'
    J2 = 'right_j2'
    J3 = 'right_j3'
    J4 = 'right_j4'
    J5 = 'right_j5'
    J6 = 'right_j6'


class Camera(str, Enum):
    """Cameras available on the Sawyer."""
    HEAD = 'head'   # head-mounted camera
    HAND = 'hand'   # wrist / end-effector camera


class Limb(str, Enum):
    """Robot limbs (Sawyer only has a right arm)."""
    RIGHT = 'right'


class GripperState(str, Enum):
    """High-level gripper state."""
    OPEN    = 'open'
    CLOSED  = 'closed'
    UNKNOWN = 'unknown'
