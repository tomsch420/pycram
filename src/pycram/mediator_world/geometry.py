from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from functools import lru_cache

from visualization_msgs.msg import Marker

from ..datastructures.dataclasses import Color
from .pose import Vector3, Pose, PoseStamped
from ..ros import Duration

class IDGenerator:
    def __init__(self):
        self._counter = 0

    @lru_cache(maxsize=None)
    def __call__(self, obj):
        self._counter += 1
        return self._counter
id_generator = IDGenerator()


@dataclass
class Shape(ABC):
    """
    Base class for all shapes in the world.
    """
    pose: PoseStamped = field(default_factory=Pose)

    def ros_message(self) -> Marker:
        """
        Returns a visualization_msgs.msg.Marker representation of the shape.
        """
        marker = Marker()
        marker.header = self.pose.header.ros_message()
        marker.action = Marker.ADD
        marker.pose = self.pose.pose.ros_message()
        marker.scale = Vector3(1., 1., 1.).ros_message()
        marker.color = Color().ros_message()
        marker.lifetime = Duration(1)
        marker.ns = "shapes"
        marker.id = id_generator(id(self))
        return marker

@dataclass
class Mesh(Shape):
    """
    A mesh shape.
    """
    filename: str = ""

    scale: Vector3 = field(default_factory=Vector3)
    """
    Scale of the mesh.
    """

    def ros_message(self) -> Marker:
        marker = super().ros_message()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "file://" + self.filename
        marker.mesh_use_embedded_materials = True
        marker.scale = self.scale.ros_message()
        return marker


@dataclass
class Primitive(Shape):
    """
    A primitive shape.
    """
    color: Color = field(default_factory=Color)

    def ros_message(self) -> Marker:
        marker = super().ros_message()
        marker.color = self.color.ros_message()
        return marker

@dataclass
class Sphere(Primitive):
    """
    A sphere shape.
    """

    radius: float = 0.5
    """
    Radius of the sphere.
    """

    def ros_message(self) -> Marker:
        marker = super().ros_message()
        marker.type = Marker.SPHERE
        marker.scale = Vector3(self.radius * 2, self.radius * 2, self.radius * 2).ros_message()
        return marker

@dataclass
class Capsule(Sphere):
    """
    A capsule shape.
    """

    length: float = 1.0
    """
    Length of the capsule.
    """


@dataclass
class Cylinder(Capsule):
    """
    A cylinder shape.
    """

    def ros_message(self) -> Marker:
        marker = super().ros_message()
        marker.type = Marker.CYLINDER
        marker.scale = Vector3(self.radius * 2, self.radius * 2, self.length).ros_message()
        return marker


@dataclass
class Box(Primitive):
    """
    A box shape.
    Pivot point is at the center of the box.
    """
    length: float = 1.0
    """
    Length of the box.
    """

    width: float = 1.0
    """
    Width of the box. 
    """

    height: float = 1.0
    """
    Height of the box.
    """

    def ros_message(self) -> Marker:
        marker = super().ros_message()
        marker.type = Marker.CUBE
        marker.scale = Vector3(self.length, self.width, self.height).ros_message()
        return marker
