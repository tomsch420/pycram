from __future__ import annotations

import datetime
from dataclasses import dataclass, field

from geometry_msgs.msg import Vector3 as ROSVector3, Quaternion as ROSQuaternion, Pose as ROSPose, Point as ROSPoint, PoseStamped as ROSPoseStamped
from std_msgs.msg import Header as ROSHeader
from rospy.rostime import Time as ROSTime
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

    def ros_message(self) -> ROSVector3:
        return ROSVector3(x=self.x, y=self.y, z=self.z)


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
        Normalize the quaternion in-place.
        """
        norm = (self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2) ** 0.5
        self.x /= norm
        self.y /= norm
        self.z /= norm
        self.w /= norm

    def ros_message(self) -> ROSQuaternion:
        return ROSQuaternion(x=self.x, y=self.y, z=self.z, w=self.w)


@dataclass
class Pose:
    """
    A pose in 3D space.
    """
    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)

    def __repr__(self):
        return (f"Pose: {[round(v, 3) for v in [self.position.x, self.position.y, self.position.z]]}, "
                f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]]}")

    def ros_message(self) -> ROSPose:
        point = ROSPoint(x=self.position.x, y=self.position.y, z=self.position.z)
        return ROSPose(position=point, orientation=self.orientation.ros_message())


@dataclass
class Header:
    """
    A header with a timestamp.
    """
    frame: str = "map"
    timestamp: datetime.datetime = field(default_factory=datetime.datetime.now)
    sequence: int = 0

    def ros_message(self) -> ROSHeader:
        stamp = ROSTime.from_sec(self.timestamp.timestamp())
        return ROSHeader(frame_id="map", stamp=stamp, seq=self.sequence)


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
                f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]]} "
                f"in frame {self.frame}")

    def ros_message(self) -> ROSPoseStamped:
        return ROSPoseStamped(pose=self.pose.ros_message(), header=self.header.ros_message())

