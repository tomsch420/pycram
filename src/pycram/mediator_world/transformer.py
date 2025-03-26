import typing

import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from tf2_ros import Buffer
from transforms3d.quaternions import quat2mat, mat2quat
from typing_extensions import Optional, Iterable, Tuple, List, TYPE_CHECKING

from .pose import Pose, PoseStamped, Header
from ..ros import Duration, Time

if TYPE_CHECKING:
    from .mediator_world import Link


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

class LocalTransformer(Buffer):
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

    world_name: str
    """
    The name of the world.
    """

    def __init__(self, world_name: str):
        super().__init__(cache_time=Duration(10))
        self.world_name = world_name
        self.registration.add(PoseStamped, do_transform_pose_stamped)

    def update_transform_for_link(self, link_origin: PoseStamped, link_name: str, time_of_update: Time):
        """
        Updates the transform for the given link frame_id.

        """
        # convert to ros messages
        link_pose_ros = link_origin.ros_message()
        link_pose_ros.header.stamp = time_of_update

        # assemble transformation
        link_transform = Transform(translation=link_pose_ros.pose.position,
                                   rotation=link_pose_ros.pose.orientation)
        link_transform = TransformStamped(header=link_pose_ros.header, child_frame_id=link_name,
                                            transform=link_transform)

        # update local transformer
        self.set_transform(link_transform, self.world_name + "/local_transformer")

    def transform_pose(self, pose: PoseStamped, pose_link: 'Link', target_link: 'Link') -> PoseStamped:
        """
        Transforms a given pose to a pose in the target link frame.

        :param pose: The pose to transform
        :param pose_link: The reference link of the pose (given in the pose.header.frame_id)
        :param target_link: The target link frame

        :return: The transformed pose
        """

        if pose.frame_id == target_link.name:
            return pose

        now = Time(0)
        target_frame = target_link.name
        self.update_transform_for_link(pose_link.origin, pose_link.name, now)
        self.update_transform_for_link(target_link.origin, target_link.name, now)

        copy_pose = pose.copy()

        copy_pose.header.stamp = now

        if not self.can_transform(target_frame, pose.frame_id, now):
            raise tf2_ros.LookupException(f"Cannot transform from {pose.header.frame_id} to {target_frame}.")

        new_pose = self.transform(copy_pose, target_frame)
        return new_pose

    # def lookup_transform_from_source_to_target_frame(self, source_frame: str, target_frame: str,
    #                                                  time: Optional[Time] = None) -> Transform:
    #     """
    #     Update the transforms for all world objects then Look up for the latest known transform that transforms a point
    #      from source frame_id to target frame_id. If no time is given the last common time between the two frames is used.
    #
    #     :param source_frame: The frame_id in which the point is currently represented
    #     :param target_frame: The frame_id in which the point should be represented
    #     :param time: Time at which the transform should be looked up
    #     :return: The transform from source_frame to target_frame
    #     """
    #     objects = list(map(self.get_object_from_frame, [source_frame, target_frame]))
    #     self.update_transforms_for_objects([obj for obj in objects if obj is not None])
    #
    #     tf_time = time if time else self.get_latest_common_time(source_frame, target_frame)
    #     translation, rotation = self.lookup_transform(source_frame, target_frame, tf_time)
    #     return Transform(translation, rotation, source_frame, target_frame)

    def get_all_frames(self) -> List[str]:
        """
        :return: A list of all known coordinate frames as a list with human-readable entries.
        """
        frames = self.all_frames_as_string().split("\n")
        frames.remove("")
        return frames
