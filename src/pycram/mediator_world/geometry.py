from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field

from typing_extensions import Self

from pycram.datastructures.dataclasses import Color


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


@dataclass
class Shape(ABC):
    """
    Base class for all shapes in the world.
    """
    pose: Pose = field(default_factory=Pose)


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


@dataclass
class Primitive(Shape):
    """
    A primitive shape.
    """
    color: Color = field(default_factory=Color)


@dataclass
class Sphere(Primitive):
    """
    A sphere shape.
    """

    radius: float = 0.5
    """
    Radius of the sphere.
    """


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


@dataclass
class Box(Primitive):
    """
    A box shape.
    """
    length: float = 1.0
    """
    Length of the box. This is the length of the x axis of the box measured from the origin.
    """

    width: float = 1.0
    """
    Width of the box. This is the width of the y axis of the box measured from the origin.
    """

    height: float = 1.0
    """
    Height of the box. This is the height of the z axis of the box measured from the origin.
    """
