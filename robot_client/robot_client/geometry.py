"""
geometry.py — strongly-typed spatial primitives for the robot client.

Types:
    Position    — 3D cartesian point (x, y, z) in metres
    Quaternion  — unit quaternion orientation (x, y, z, w)
    Euler       — roll/pitch/yaw in radians (XYZ convention)
    Pose        — position + orientation
    JointAngles — ordered joint angles for a 7-DOF arm

All types are dataclasses, JSON-serialisable (via to_dict/from_dict),
and use scipy.spatial.transform.Rotation for angle conversions.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict, field
from typing import List, Sequence

import numpy as np
from scipy.spatial.transform import Rotation


# ── Position ─────────────────────────────────────────────────────────────────

@dataclass
class Position:
    """3D cartesian position in metres."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_list(cls, v: Sequence[float]) -> Position:
        return cls(float(v[0]), float(v[1]), float(v[2]))

    @classmethod
    def from_dict(cls, d: dict) -> Position:
        return cls(float(d['x']), float(d['y']), float(d['z']))

    def to_dict(self) -> dict:
        return {'x': self.x, 'y': self.y, 'z': self.z}

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def __add__(self, other: Position) -> Position:
        return Position(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Position) -> Position:
        return Position(self.x - other.x, self.y - other.y, self.z - other.z)

    def distance_to(self, other: Position) -> float:
        return float(np.linalg.norm(self.to_array() - other.to_array()))

    def __repr__(self) -> str:
        return f"Position(x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f})"


# ── Quaternion ────────────────────────────────────────────────────────────────

@dataclass
class Quaternion:
    """
    Unit quaternion orientation.

    Convention: (x, y, z, w) — same as ROS / Eigen.
    Internally delegates math to scipy.spatial.transform.Rotation.
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    # ── constructors ─────────────────────────────────────────────────────────

    @classmethod
    def identity(cls) -> Quaternion:
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Create from roll/pitch/yaw in radians (XYZ extrinsic convention)."""
        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        x, y, z, w = r.as_quat()   # scipy uses (x, y, z, w)
        return cls(float(x), float(y), float(z), float(w))

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> Quaternion:
        """Create from a 3×3 rotation matrix."""
        r = Rotation.from_matrix(matrix)
        x, y, z, w = r.as_quat()
        return cls(float(x), float(y), float(z), float(w))

    @classmethod
    def from_list(cls, v: Sequence[float]) -> Quaternion:
        """From [x, y, z, w] list."""
        return cls(float(v[0]), float(v[1]), float(v[2]), float(v[3]))

    @classmethod
    def from_dict(cls, d: dict) -> Quaternion:
        return cls(float(d['x']), float(d['y']), float(d['z']), float(d['w']))

    # ── conversions ───────────────────────────────────────────────────────────

    def _rot(self) -> Rotation:
        return Rotation.from_quat([self.x, self.y, self.z, self.w])

    def to_euler(self) -> Euler:
        """Return roll/pitch/yaw in radians (XYZ extrinsic)."""
        roll, pitch, yaw = self._rot().as_euler('xyz')
        return Euler(float(roll), float(pitch), float(yaw))

    def to_matrix(self) -> np.ndarray:
        """Return 3×3 rotation matrix."""
        return self._rot().as_matrix()

    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.w]

    def to_dict(self) -> dict:
        return {'x': self.x, 'y': self.y, 'z': self.z, 'w': self.w}

    def normalized(self) -> Quaternion:
        arr = np.array([self.x, self.y, self.z, self.w])
        arr /= np.linalg.norm(arr)
        return Quaternion(float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))

    def __mul__(self, other: Quaternion) -> Quaternion:
        """Quaternion composition (self * other)."""
        r = self._rot() * other._rot()
        x, y, z, w = r.as_quat()
        return Quaternion(float(x), float(y), float(z), float(w))

    def __repr__(self) -> str:
        rpy = self.to_euler()
        return (f"Quaternion(x={self.x:.4f}, y={self.y:.4f}, "
                f"z={self.z:.4f}, w={self.w:.4f})  "
                f"[rpy=({rpy.roll:.3f}, {rpy.pitch:.3f}, {rpy.yaw:.3f}) rad]")


# ── Euler ─────────────────────────────────────────────────────────────────────

@dataclass
class Euler:
    """Roll / pitch / yaw in radians (XYZ extrinsic convention)."""
    roll:  float = 0.0
    pitch: float = 0.0
    yaw:   float = 0.0

    @classmethod
    def from_degrees(cls, roll: float, pitch: float, yaw: float) -> Euler:
        return cls(np.radians(roll), np.radians(pitch), np.radians(yaw))

    @classmethod
    def from_dict(cls, d: dict) -> Euler:
        return cls(float(d['roll']), float(d['pitch']), float(d['yaw']))

    def to_quaternion(self) -> Quaternion:
        return Quaternion.from_euler(self.roll, self.pitch, self.yaw)

    def to_degrees(self) -> Euler:
        return Euler(np.degrees(self.roll), np.degrees(self.pitch), np.degrees(self.yaw))

    def to_dict(self) -> dict:
        return {'roll': self.roll, 'pitch': self.pitch, 'yaw': self.yaw}

    def __repr__(self) -> str:
        d = self.to_degrees()
        return (f"Euler(roll={self.roll:.3f}, pitch={self.pitch:.3f}, yaw={self.yaw:.3f} rad)  "
                f"[{d.roll:.1f}°, {d.pitch:.1f}°, {d.yaw:.1f}°]")


# ── Pose ──────────────────────────────────────────────────────────────────────

@dataclass
class Pose:
    """6-DOF end-effector pose: position (m) + orientation (quaternion)."""
    position:    Position   = field(default_factory=Position)
    orientation: Quaternion = field(default_factory=Quaternion.identity)

    @classmethod
    def from_dict(cls, d: dict) -> Pose:
        return cls(
            position=Position.from_dict(d['position']),
            orientation=Quaternion.from_dict(d['orientation']),
        )

    def to_dict(self) -> dict:
        return {
            'position':    self.position.to_dict(),
            'orientation': self.orientation.to_dict(),
        }

    @classmethod
    def from_position_euler(cls, x: float, y: float, z: float,
                            roll: float = 0.0, pitch: float = 0.0,
                            yaw: float = 0.0) -> Pose:
        """Convenience constructor from xyz + rpy (radians)."""
        return cls(
            position=Position(x, y, z),
            orientation=Quaternion.from_euler(roll, pitch, yaw),
        )

    def to_matrix(self) -> np.ndarray:
        """Return 4×4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.orientation.to_matrix()
        T[:3,  3] = self.position.to_array()
        return T

    @classmethod
    def from_matrix(cls, T: np.ndarray) -> Pose:
        return cls(
            position=Position.from_list(T[:3, 3]),
            orientation=Quaternion.from_matrix(T[:3, :3]),
        )

    def __repr__(self) -> str:
        rpy = self.orientation.to_euler()
        return (f"Pose(pos=({self.position.x:.3f}, {self.position.y:.3f}, {self.position.z:.3f}) m  "
                f"rpy=({rpy.roll:.3f}, {rpy.pitch:.3f}, {rpy.yaw:.3f}) rad)")


# ── JointAngles ───────────────────────────────────────────────────────────────

@dataclass
class JointAngles:
    """
    Joint angles for a 7-DOF arm (Sawyer: j0 … j6) in radians.

    Can be constructed from a plain list and converts back to one for transport.
    """
    j0: float = 0.0
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    j5: float = 0.0
    j6: float = 0.0

    @classmethod
    def from_list(cls, angles: Sequence[float]) -> JointAngles:
        if len(angles) != 7:
            raise ValueError(f"Expected 7 joint angles, got {len(angles)}")
        return cls(*[float(a) for a in angles])

    @classmethod
    def from_dict(cls, d: dict) -> JointAngles:
        return cls(
            float(d.get('j0', 0)), float(d.get('j1', 0)),
            float(d.get('j2', 0)), float(d.get('j3', 0)),
            float(d.get('j4', 0)), float(d.get('j5', 0)),
            float(d.get('j6', 0)),
        )

    def to_list(self) -> List[float]:
        return [self.j0, self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]

    def to_dict(self) -> dict:
        return asdict(self)

    def to_array(self) -> np.ndarray:
        return np.array(self.to_list())

    def __repr__(self) -> str:
        vals = ', '.join(f'j{i}:{v:.3f}' for i, v in enumerate(self.to_list()))
        return f"JointAngles({vals})"
