from __future__ import annotations

import atexit
import enum
import threading
import time
from dataclasses import dataclass, field
from typing import List

from typing_extensions import Set, Optional
from visualization_msgs.msg import MarkerArray

from .geometry import Shape
from .pose import PoseStamped
from .transformer import LocalTransformer
from ..datastructures.enums import JointType
from ..ros import create_publisher


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


class WorldPublisher:
    """
    Publishes the visuals of every link in the world to a ros topic.
    """

    world: World
    """
    The world to read the links from.
    """

    topic_name: str
    """
    The name of the topic to publish the visuals to.
    """

    interval: float
    """
    The interval in seconds at which to publish the visuals.
    """

    def __init__(self, world: World, topic_name="/pycram/viz_marker", interval=0.1, reference_frame="map"):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval
        self.reference_frame = reference_frame

        self.pub = create_publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish, name="WorldPublisher")

        self.kill_event = threading.Event()
        self.world = world
        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            marker_array = self._make_marker_array()
            self.pub.publish(marker_array)
            time.sleep(self.interval)

    def _make_marker_array(self) -> MarkerArray:
        """
        Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
        creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
        tag of the URDF.

        :return: An Array of Visualization Marker
        """
        marker_array = MarkerArray()
        for link in self.world.links:
            for shape in link.visual:
                marker = shape.ros_message()
                marker_array.markers.append(marker)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()
