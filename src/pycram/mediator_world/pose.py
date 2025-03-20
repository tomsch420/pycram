from __future__ import annotations

import datetime
from dataclasses import dataclass, field
from typing_extensions import Self


@dataclass
class Vector3:
    """
    A 3D vector with x, y and z coordinates.
    """

    x: float = 0
    y: float = 0
    z: float = 0

    def euclidean_distance(self, other: Self) -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5


@dataclass
class Quaternion:
    """
    A quaternion with x, y, z and w components.
    """

    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 1

    def __post_init__(self):
        self.normalize()

    def normalize(self):
        """
        Normalize the quaternion.
        """
        norm = (self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2) ** 0.5
        self.x /= norm
        self.y /= norm
        self.z /= norm
        self.w /= norm


@dataclass
class Pose:
    """
    A pose in 3D space.
    """
    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)

    def __repr__(self):
        return (f"Pose: {[round(v, 3) for v in [self.position.x, self.position.y, self.position.z]]}, "
                f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z]]}")


@dataclass
class Header:
    """
    A header with a timestamp.
    """
    frame: str = "map"
    timestamp: datetime.datetime = field(default_factory=datetime.datetime.now)
    sequence: int = 0


@dataclass
class PoseStamped:
    """
    A pose in 3D space with a timestamp.
    """
    pose: Pose = field(default_factory=Pose)
    header: Header = field(default_factory=Header)

    @property
    def position(self):
        return self.pose.position

    @property
    def orientation(self):
        return self.pose.orientation

    @property
    def frame(self):
        return self.header.frame

    def __repr__(self):
        return (f"Pose: {[round(v, 3) for v in [self.position.x, self.position.y, self.position.z]]}, "
                f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z]]} "
                f"in frame {self.frame}")
