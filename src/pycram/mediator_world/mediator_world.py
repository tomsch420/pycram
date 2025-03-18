from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from typing_extensions import Set, Optional

from ..datastructures.enums import JointType
from .geometry import Shape, Pose


@dataclass
class WorldEntity:
    """
    Base class for all entities in the world.
    """
    _world: Optional[World] = field(repr=False, default=None, init=False)


@dataclass
class Link(WorldEntity):
    """
    Represents a link in the world.
    """
    name: str

    pose: Pose
    """
    The pose of the link in the world.
    """

    visual: List[Shape] = field(default_factory=list)
    """
    List of shapes that represent the visual appearance of the link.
    The poses of the shapes are relative to the link.
    """

    collision: List[Shape] = field(default_factory=list)
    """
    List of shapes that represent the collision geometry of the link.
    The poses of the shapes are relative to the link.
    """

    def __hash__(self):
        return hash(self.name)


@dataclass
class Joint(WorldEntity):
    """
    Represents a joint in the world.
    """
    type: JointType
    parent: Link
    child: Link

    def __hash__(self):
        return hash((self.parent, self.child))


@dataclass
class World:
    """
    Represents the world (belief-state) in PyCRAM.
    This class implements a mediator pattern.
    """

    links: Set[Link] = field(default_factory=set)
    """
    Set of links in the world.
    """

    joints: Set[Joint] = field(default_factory=set)
    """
    Set of joints in the world.
    """

    def add_link(self, link: Link):
        """
        Adds a link to the world.

        :param link: The link to add.
        """
        link._world = self
        self.links.add(link)

    def add_joint(self, joint: Joint):
        """
        Adds a joint to the world.

        :param joint: The joint to add.
        """
        self.add_link(joint.parent)
        self.add_link(joint.child)
        joint._world = self
        self.joints.add(joint)
