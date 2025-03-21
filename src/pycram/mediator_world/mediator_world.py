from __future__ import annotations

from dataclasses import dataclass, field
from enum import Flag, auto
from typing import List

from typing_extensions import Set, Optional

from ..datastructures.enums import JointType
from .geometry import Shape
from .pose import Vector3, Pose, PoseStamped


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

    pose: Optional[PoseStamped]
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

    @property
    def child_links(self):
        """
        Returns all links that are child links of this link.
        """
        return {joint.child for joint in self._world.joints if joint.parent == self}

    @property
    def recursive_child_links(self):
        """
        Returns all links that are child links of this link, recursively.
        """
        child_links = self.child_links
        for child_link in child_links:
            child_links |= child_link.recursive_child_links
        return child_links

class LinkView(WorldEntity):
    """
    Represents a view on a set of links in the world.
    """
    ...

class JointAxis(Flag):
    """
    Flag for axis identifiers used in Joints.
    """
    X = auto()
    Y = auto()
    Z = auto()


@dataclass
class Joint(WorldEntity):
    """
    Represents a joint in the world.
    """
    type: JointType
    """
    The type of the joint.
    """

    parent: Link
    """
    The parent link of the joint.
    """

    child: Link
    """
    The child link of the joint.
    """

    axis: JointAxis
    """
    The axis (perhaps multiple) of the joint.
    """
    value: float = 0.

    lower_limit: Optional[float] = None
    """
    The lower limit of the joint.
    """

    upper_limit: Optional[float] = None
    """
    The upper limit of the joint.
    """

    damping: float = 0.
    """
    The damping of the joint.
    """

    friction: float = 0.
    """
    The friction of the joint.
    """

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

    def add_from_world(self, world: World):
        """
        Adds all links and joints from another world to this world.

        :param world: The world to add from.
        """
        for link in world.links:
            self.add_link(link)
        for joint in world.joints:
            self.add_joint(joint)

    def get_link_by_name(self, name: str) -> Optional[Link]:
        """
        Returns the link with the given name.

        :param name: The name of the link.
        :return: The link with the given name or None if not found.
        """
        for link in self.links:
            if link.name == name:
                return link
        return None