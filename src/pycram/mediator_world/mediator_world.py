from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import List

from typing_extensions import Set, Optional

from .geometry import Shape
from .pose import PoseStamped
from .transformer import LocalTransformer
from ..datastructures.enums import JointType


@dataclass
class WorldEntity:
    """
    Base class for all entities in the world.
    """

    _world: Optional[World] = field(repr=False, default=None, init=False)
    """
    The world this entity is part of. This is not managed by the entity itself.
    """

@dataclass
class Link(WorldEntity):
    """
    Represents a link in the world.
    """

    name: str
    """
    The name of the link. Must be unique in the world.
    """

    origin: PoseStamped = PoseStamped()
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

    @property
    def parent_link(self):
        """
        Returns the parent link of this link.
        """
        for joint in self._world.joints:
            if joint.child == self:
                return joint.parent
        return None

    @classmethod
    def from_link(cls, link: Link):
        """
        Creates a new link from an existing link.
        """
        new_link = cls(link.name, link.origin, link.visual, link.collision)
        new_link._world = link._world
        return new_link

    def __eq__(self, other):
        return self.name == other.name and self._world is other._world


class LinkView(WorldEntity):
    """
    Represents a view on a set of links in the world.
    """
    ...


class JointAxis(int, enum.Enum):
    """
    Enum for axis identifiers used in Joints.
    PyCRAM currently does not support joints over multiple axis (ball joint, etc.)
    """
    X = 0
    Y = 1
    Z = 2


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

    origin: PoseStamped = PoseStamped()
    """
    The origin of the joint.
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

    _transformer: LocalTransformer = field(init=False)
    """
    The local transformer for the world.
    """

    origin: Link = field(default=Link("map"))
    """
    The origin link of the world.
    """

    def __post_init__(self):
        self._transformer = LocalTransformer(self.origin.name)

        self.add_link(self.origin)
        [self.add_link(link) for link in self.links]
        [self.add_joint(joint) for joint in self.joints]
        self.transform_everything_to_frame(self.origin)

    def add_link(self, link: Link):
        """
        Adds a link to the world.

        :param link: The link to add.
        """
        link._world = self
        self.links.add(link)
        self.transform_everything_to_frame(self.origin)

    def add_joint(self, joint: Joint):
        """
        Adds a joint to the world.

        :param joint: The joint to add.
        """
        joint._world = self
        self.joints.add(joint)
        self.add_link(joint.parent)
        self.add_link(joint.child)

    def add_from_world(self, world: World):
        """
        Adds all links and joints from another world to this world.

        :param world: The world to add from.
        """
        for joint in world.joints:
            self.add_joint(joint)
        for link in world.links:
            self.add_link(link)

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

    def transform_everything_to_frame(self, frame: Link):
        """
        Transforms all poses that are contained in the world to the given frame.

        :param frame: The frame to transform to.
        """
        for link in self.links:
            link.origin = self._transformer.transform_pose(link.origin, link, frame)
            for shape in link.visual:
                shape.origin = self._transformer.transform_pose(shape.origin, link, frame)
            for shape in link.collision:
                shape.origin = self._transformer.transform_pose(shape.origin, link, frame)
        for joint in self.joints:
            joint.origin = self._transformer.transform_pose(joint.origin, joint.parent, frame)

