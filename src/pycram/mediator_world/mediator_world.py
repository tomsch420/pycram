from __future__ import annotations

import atexit
import enum
import threading
import time
from dataclasses import dataclass, field
from functools import cached_property
from typing import List

import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from tf2_ros import Buffer
from transforms3d.quaternions import mat2quat, quat2mat
from typing_extensions import Set, Optional, Tuple, Iterable
from visualization_msgs.msg import MarkerArray

from .geometry import Shape
from .pose import Pose, PoseStamped, Header
from ..datastructures.enums import JointType
from ..ros import create_publisher, Duration, Time


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


def _build_affine(
        rotation: Optional[Iterable] = None,
        translation: Optional[Iterable] = None) -> np.ndarray:
    """
    Build an affine matrix from a quaternion and a translation.

    :param rotation: The quaternion as [w, x, y, z]
    :param translation: The translation as [x, y, z]
    :returns: The quaternion and the translation array
    """
    affine = np.eye(4)
    if rotation is not None:
        affine[:3, :3] = quat2mat(np.asarray(rotation))
    if translation is not None:
        affine[:3, 3] = np.asarray(translation)
    return affine

def _transform_to_affine(transform: TransformStamped) -> np.ndarray:
    """
    Convert a `TransformStamped` to a affine matrix.

    :param transform: The transform that should be converted
    :returns: The affine transform
    """
    transform = transform.transform
    transform_rotation_matrix = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z
    ]
    transform_translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ]
    return _build_affine(transform_rotation_matrix, transform_translation)


def _decompose_affine(affine: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decompose an affine transformation into a quaternion and the translation.

    :param affine: The affine transformation matrix
    :returns: The quaternion and the translation array
    """
    return mat2quat(affine[:3, :3]), affine[:3, 3]

# Pose
def do_transform_pose(
        pose: Pose,
        transform: TransformStamped) -> Pose:
    """
    Transform a `Pose` using a given `TransformStamped`.

    This method is used to share the tranformation done in
    `do_transform_pose_stamped()` and `do_transform_pose_with_covariance_stamped()`

    :param pose: The pose
    :param transform: The transform
    :returns: The transformed pose
    """
    quaternion, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(
                translation=[
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ],
                rotation=[
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z])))
    res = Pose()
    res.position.x = point[0]
    res.position.y = point[1]
    res.position.z = point[2]
    res.orientation.w = quaternion[0]
    res.orientation.x = quaternion[1]
    res.orientation.y = quaternion[2]
    res.orientation.z = quaternion[3]
    return res


# PoseStamped
def do_transform_pose_stamped(
        pose: PoseStamped,
        transform: TransformStamped) -> PoseStamped:
    """
    Transform a `PoseStamped` using a given `TransformStamped`.

    :param pose: The stamped pose
    :param transform: The transform
    :returns: The transformed pose stamped
    """
    res = PoseStamped()
    res.pose = do_transform_pose(pose.pose, transform)
    res.header = Header(transform.header.frame_id, pose.header.timestamp)
    return res

class LocalTransformer(Buffer, WorldEntity):
    """
    This class allows to use the TF class TransformerROS without using the ROS
    network system or the topic /tf, where transforms are usually published to.
    Instead, a local transformer is saved and allows to publish local transforms,
    as well the use of TFs convenient lookup functions (see functions below).

    This class uses the robots (currently only one! supported) URDF file to
    initialize the tfs for the robot. Moreover, the function update_local_transformer_from_btr
    updates these tfs by copying the tfs state from the world.

    This class extends the TransformerRos, you can find documentation for TransformerROS here:
    `TFDoc <http://wiki.ros.org/tf/TfUsingPython>`_
    """

    def __init__(self):
        super().__init__(cache_time=Duration(10))
        # Since this file can't import world.py this holds the reference to the current_world
        self._world = None
        self.registration.add(PoseStamped, do_transform_pose_stamped)

    def transform_pose_to_link_frame(self, pose: PoseStamped, link: Link) -> PoseStamped:
        """
        Transforms the given pose to the coordinate frame_id of the given link.
        """
        return self.transform_pose(pose, link.name)

    def update_transform_for_link(self, link: Link, time_of_update: Time):
        """
        Updates the transform for the given link frame_id.

        """
        if link == self._world.origin:
            return

        # convert to ros messages
        link_pose_ros = link.origin.ros_message()
        link_pose_ros.header.stamp = time_of_update

        # assemble transformation
        link_transform = Transform(translation=link_pose_ros.pose.position,
                                   rotation=link_pose_ros.pose.orientation)
        link_transform = TransformStamped(header=link_pose_ros.header, child_frame_id=link.name,
                                            transform=link_transform)

        # update local transformer
        self.set_transform(link_transform, self._world.origin.origin.frame_id + "/local_transformer")


    def transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        Transforms a given pose to the target frame_id after updating the transforms for all objects in the current world.

        :param pose: Pose that should be transformed
        :param target_frame: Name of the TF frame_id into which the Pose should be transformed
        :return: A transformed pose in the target frame_id
        """

        now = Time().now()

        self.update_transform_for_link(self._world.get_link_by_name(pose.frame_id), now)
        self.update_transform_for_link(self._world.get_link_by_name(target_frame), now)

        copy_pose = pose.copy()

        copy_pose.header.stamp = now

        if not self.can_transform(target_frame, pose.frame_id, now):
            raise tf2_ros.LookupException(f"Cannot transform from {pose.header.frame_id} to {target_frame}.")

        new_pose = self.transform(copy_pose, target_frame)
        return new_pose

    def lookup_transform_from_source_to_target_frame(self, source_frame: str, target_frame: str,
                                                     time: Optional[Time] = None) -> Transform:
        """
        Update the transforms for all world objects then Look up for the latest known transform that transforms a point
         from source frame_id to target frame_id. If no time is given the last common time between the two frames is used.

        :param source_frame: The frame_id in which the point is currently represented
        :param target_frame: The frame_id in which the point should be represented
        :param time: Time at which the transform should be looked up
        :return: The transform from source_frame to target_frame
        """
        objects = list(map(self.get_object_from_frame, [source_frame, target_frame]))
        self.update_transforms_for_objects([obj for obj in objects if obj is not None])

        tf_time = time if time else self.get_latest_common_time(source_frame, target_frame)
        translation, rotation = self.lookup_transform(source_frame, target_frame, tf_time)
        return Transform(translation, rotation, source_frame, target_frame)

    def get_all_frames(self) -> List[str]:
        """
        :return: A list of all known coordinate frames as a list with human-readable entries.
        """
        frames = self.all_frames_as_string().split("\n")
        frames.remove("")
        return frames


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

    transformer: LocalTransformer = LocalTransformer()
    """
    The local transformer for the world.
    """

    origin: Link = field(default=Link("map"))
    """
    The origin link of the world.
    """

    def __post_init__(self):
        self.transformer._world = self
        self.add_link(self.origin)

        [self.add_link(link) for link in self.links]
        [self.add_joint(joint) for joint in self.joints]

    def add_link(self, link: Link):
        """
        Adds a link to the world.

        :param link: The link to add.
        """
        if link._world == self:
            return
        link._world = self
        self.links.add(link)

        self.transformer.update_transform_for_link(link, Time().now())


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
                transformed_pose: PoseStamped = self.world.transformer.transform_pose(shape.origin,
                                                                                      self.world.origin.origin.frame_id)
                transformed_pose_ros = transformed_pose.ros_message()
                marker = shape.ros_message()
                marker.pose = transformed_pose_ros.pose
                marker.header = transformed_pose_ros.header
                marker_array.markers.append(marker)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()
