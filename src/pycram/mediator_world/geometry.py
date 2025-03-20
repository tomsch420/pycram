from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field

from pycram.datastructures.dataclasses import Color
from pycram.mediator_world.pose import Vector3, Pose, PoseStamped


@dataclass
class Shape(ABC):
    """
    Base class for all shapes in the world.
    """
    pose: PoseStamped = field(default_factory=Pose)


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
