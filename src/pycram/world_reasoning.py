import numpy as np
from typing_extensions import List, Tuple, Optional, Union, Dict

from .datastructures.dataclasses import ContactPointsList
from .datastructures.pose import Pose, Transform
from .datastructures.world import World, UseProspectionWorld
from .external_interfaces.ik import try_to_reach, try_to_reach_with_grasp
from .robot_description import RobotDescription
from .ros.logging import logdebug
from .utils import RayTestUtils
from .world_concepts.world_object import Object, Link
from .config import world_conf as conf


def stable(obj: Object) -> bool:
    """
    Checks if an object is stable in the world. Stable meaning that it's position will not change after simulating
    physics in the World. This will be done by simulating the world for 10 seconds and compare
    the previous coordinates with the coordinates after the simulation.

    :param obj: The object which should be checked
    :return: True if the given object is stable in the world False else
    """
    prospection_obj = World.current_world.get_prospection_object_for_object(obj)
    with UseProspectionWorld():
        coords_prev = prospection_obj.get_position_as_list()
        World.current_world.set_gravity([0, 0, -9.8])

        World.current_world.simulate(2)
        coords_past = prospection_obj.get_position_as_list()

        coords_prev = list(map(lambda n: round(n, 3), coords_prev))
        coords_past = list(map(lambda n: round(n, 3), coords_past))
        return coords_past == coords_prev


def contact(
        object1: Object,
        object2: Object,
        return_links: bool = False) -> Union[bool, Tuple[bool, List[Tuple[Link, Link]]]]:
    """
    Checks if two objects are in contact or not. If the links should be returned then the output will also contain a
    list of tuples where the first element is the link name of 'object1' and the second element is the link name of
    'object2'.

    :param object1: The first object
    :param object2: The second object
    :param return_links: If the respective links on the objects that are in contact should be returned.
    :return: True if the two objects are in contact False else. If links should be returned a list of links in contact
    """

    with UseProspectionWorld():
        prospection_obj1 = World.current_world.get_prospection_object_for_object(object1)
        prospection_obj2 = World.current_world.get_prospection_object_for_object(object2)
        World.current_world.perform_collision_detection()
        con_points: ContactPointsList = World.current_world.get_contact_points_between_two_objects(prospection_obj1,
                                                                                                   prospection_obj2)
        objects_are_in_contact = len(con_points) > 0
        if return_links:
            contact_links = [(point.link_a, point.link_b) for point in con_points]
            return objects_are_in_contact, contact_links
        else:
            return objects_are_in_contact


def prospect_robot_contact(robot: Object, pose: Pose,
                           ignore_collision_with: Optional[List[Object]] = None) -> bool:
    """
    Check if the robot collides with any object in the world at the given pose.

    :param robot: The robot object
    :param pose: The pose to check for collision
    :param ignore_collision_with: A list of objects to ignore collision with
    :return: True if the robot collides with any object, False otherwise
    """
    with UseProspectionWorld():
        prospection_robot = World.current_world.get_prospection_object_for_object(robot)
        prospection_robot.set_pose(pose)
        floor = prospection_robot.world.get_object_by_name("floor")
        ignore_collision_with = [] if ignore_collision_with is None else ignore_collision_with
        ignore = [o.name for o in ignore_collision_with]
        for obj in prospection_robot.world.objects:
            if obj.name in ([prospection_robot.name, floor.name] + ignore):
                continue
            in_contact, contact_links = contact(prospection_robot, obj, return_links=True)
            if in_contact and not is_held_object(prospection_robot, obj, [links[0] for links in contact_links]):
                logdebug(f"Robot is in contact with {obj.name} in prospection: {obj.world.is_prospection_world}"
                         f"at position {pose.position_as_list()} and z_angle {pose.z_angle}")
                return True
            logdebug(f"Robot is not in contact with {obj.name} in prospection: {obj.world.is_prospection_world}"
                     f"at position {pose.position_as_list()} and z_angle {pose.z_angle}")
    return False


def is_held_object(robot: Object, obj: Object, robot_contact_links: List[Link]) -> bool:
    """
    Check if the object is picked by the robot.

    :param robot: The robot object
    :param obj: The object to check if it is picked
    :param robot_contact_links: The links of the robot that are in contact with the object
    :return: True if the object is picked by the robot, False otherwise
    """
    picked_object = False
    if obj in robot.attachments:
        arm_chains = RobotDescription.current_robot_description.get_manipulator_chains()
        for chain in arm_chains:
            gripper_links = chain.end_effector.links
            if (robot.attachments[obj].parent_link.name in gripper_links
                    and all(link.name in gripper_links for link in robot_contact_links)):
                picked_object = True
                continue
    return picked_object


def get_visible_objects(
        camera_pose: Pose,
        front_facing_axis: Optional[List[float]] = None,
        plot_segmentation_mask: bool = False) -> Tuple[np.ndarray, Pose]:
    """
    Return a segmentation mask of the objects that are visible from the given camera pose and the front facing axis.

    :param camera_pose: The pose of the camera in world coordinate frame.
    :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
    :param plot_segmentation_mask: If the segmentation mask should be plotted
    :return: A segmentation mask of the objects that are visible and the pose of the point at exactly 2 meters in front of the camera in the direction of the front facing axis with respect to the world coordinate frame.
    """
    if front_facing_axis is None:
        front_facing_axis = RobotDescription.current_robot_description.get_default_camera().front_facing_axis

    camera_frame = RobotDescription.current_robot_description.get_camera_frame(World.robot.name)
    world_to_cam = camera_pose.to_transform(camera_frame)

    cam_to_point = Transform(list(np.multiply(front_facing_axis, 2)), [0, 0, 0, 1], camera_frame,
                             "point")
    target_point = (world_to_cam * cam_to_point).to_pose()

    seg_mask = World.current_world.get_images_for_target(target_point, camera_pose)[2]

    if plot_segmentation_mask:
        RayTestUtils.plot_segmentation_mask(seg_mask)

    return seg_mask, target_point


def visible(
        obj: Object,
        camera_pose: Pose,
        front_facing_axis: Optional[List[float]] = None,
        threshold: float = 0.8,
        plot_segmentation_mask: bool = False) -> bool:
    """
    Checks if an object is visible from a given position. This will be achieved by rendering the object
    alone and counting the visible pixel, then rendering the complete scene and compare the visible pixels with the
    absolut count of pixels.

    :param obj: The object for which the visibility should be checked
    :param camera_pose: The pose of the camera in map frame
    :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
    :param threshold: The minimum percentage of the object that needs to be visible for this method to return true.
    :param plot_segmentation_mask: If the segmentation mask should be plotted.
    :return: True if the object is visible from the camera_position False if not
    """
    with UseProspectionWorld():
        prospection_obj = World.current_world.get_prospection_object_for_object(obj)
        if World.robot:
            prospection_robot = World.current_world.get_prospection_object_for_object(World.robot)

        state_id = World.current_world.save_state()
        for obj in World.current_world.objects:
            if obj == prospection_obj or (World.robot and obj == prospection_robot):
                continue
            else:
                obj.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]), set_attachments=False)

        seg_mask, target_point = get_visible_objects(camera_pose, front_facing_axis, plot_segmentation_mask)
        max_pixel = np.array(seg_mask == prospection_obj.id).sum()

        World.current_world.restore_state(state_id)

        if max_pixel == 0:
            # Object is not visible
            return False

        seg_mask = World.current_world.get_images_for_target(target_point, camera_pose)[2]
        real_pixel = np.array(seg_mask == prospection_obj.id).sum()

        return real_pixel / max_pixel > threshold > 0


def occluding(
        obj: Object,
        camera_pose: Pose,
        front_facing_axis: Optional[List[float]] = None,
        plot_segmentation_mask: bool = False) -> List[Object]:
    """
    Lists all objects which are occluding the given object. This works similar to 'visible'.
    First the object alone will be rendered and the position of the pixels of the object in the picture will be saved.
    After that the complete scene will be rendered and the previous saved pixel positions will be compared to the
    actual pixels, if in one pixel another object is visible ot will be saved as occluding.

    :param obj: The object for which occlusion should be checked
    :param camera_pose: The pose of the camera in world coordinate frame
    :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
    :param plot_segmentation_mask: If the segmentation mask should be plotted
    :return: A list of occluding objects
    """

    with UseProspectionWorld():
        state_id = World.current_world.save_state()
        for other_obj in World.current_world.objects:
            if other_obj.name == World.current_world.robot.name:
                continue
            elif obj.get_pose() == other_obj.get_pose():
                obj = other_obj
            else:
                other_obj.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))

        seg_mask, target_point = get_visible_objects(camera_pose, front_facing_axis, plot_segmentation_mask)

        # All indices where the object that could be occluded is in the image
        # [0] at the end is to reduce by one dimension because dstack adds an unnecessary dimension
        pix = np.dstack(np.nonzero(seg_mask == obj.id))[0]

        World.current_world.restore_state(state_id)

        occluding_obj_ids = []
        seg_mask = World.current_world.get_images_for_target(target_point, camera_pose)[2]
        for c in pix:
            if not seg_mask[c[0]][c[1]] == obj.id:
                occluding_obj_ids.append(seg_mask[c[0]][c[1]])

        occ_objects = list(set(map(World.current_world.get_object_by_id, occluding_obj_ids)))
        occ_objects = list(map(World.current_world.get_object_for_prospection_object, occ_objects))

        return occ_objects


def reachable(
        pose_or_object: Union[Object, Pose],
        robot: Object,
        gripper_name: str,
        threshold: float = 0.01) -> bool:
    """
    Checks if the robot can reach a given position. To determine this the inverse kinematics are
    calculated and applied. Afterward the distance between the position and the given end effector is calculated, if
    it is smaller than the threshold the reasoning query returns True, if not it returns False.

    :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
    :param robot: The robot that should reach for the position
    :param gripper_name: The name of the end effector
    :param threshold: The threshold between the end effector and the position.
    :return: True if the end effector is closer than the threshold to the target position, False in every other case
    """

    prospection_robot = World.current_world.get_prospection_object_for_object(robot)
    with UseProspectionWorld():
        target_pose = try_to_reach(pose_or_object, prospection_robot, gripper_name)

        if not target_pose:
            return False

        gripper_pose = prospection_robot.get_link_pose(gripper_name)
        diff = target_pose.dist(gripper_pose)

    return diff < threshold


def blocking(
        pose_or_object: Union[Object, Pose],
        robot: Object,
        gripper_name: str,
        grasp: str = None) -> Union[List[Object], None]:
    """
    Checks if any objects are blocking another object when a robot tries to pick it. This works
    similar to the reachable predicate. First the inverse kinematics between the robot and the object will be
    calculated and applied. Then it will be checked if the robot is in contact with any object except the given one.
    If the given pose or Object is not reachable None will be returned

    :param pose_or_object: The object or pose for which blocking objects should be found
    :param robot: The robot Object who reaches for the object
    :param gripper_name: The name of the end effector of the robot
    :param grasp: The grasp type with which the object should be grasped
    :return: A list of objects the robot is in collision with when reaching for the specified object or None if the pose or object is not reachable.
    """

    with UseProspectionWorld():
        prospection_robot = World.current_world.get_prospection_object_for_object(robot)
        if grasp:
            try_to_reach_with_grasp(pose_or_object, prospection_robot, gripper_name, grasp)
        else:
            try_to_reach(pose_or_object, prospection_robot, gripper_name)

        block = [World.current_world.get_object_for_prospection_object(obj) for obj in World.current_world.objects
                 if contact(prospection_robot, obj)]
    return block


def supporting(
        object1: Object,
        object2: Object) -> bool:
    """
    Checks if one object is supporting another object. An object supports another object if they are in
    contact and the second object is above the first one. (e.g. a Bottle will be supported by a table)

    :param object1: Object that is supported
    :param object2: Object that supports the first object
    :return: True if the second object is in contact with the first one and the second is above the first else False
    """
    return contact(object1, object2) and object2.get_position().z > object1.get_position().z


def link_pose_for_joint_config(
        obj: Object,
        joint_config: Dict[str, float],
        link_name: str) -> Pose:
    """
    Get the pose a link would be in if the given joint configuration would be applied to the object.
    This is done by using the respective object in the prospection world and applying the joint configuration
    to this one. After applying the joint configuration the link position is taken from there.

    :param obj: Object of which the link is a part
    :param joint_config: Dict with the goal joint configuration
    :param link_name: Name of the link for which the pose should be returned
    :return: The pose of the link after applying the joint configuration
    """
    prospection_object = World.current_world.get_prospection_object_for_object(obj)
    with UseProspectionWorld():
        for joint, pose in joint_config.items():
            prospection_object.set_joint_position(joint, pose)
        return prospection_object.get_link_pose(link_name)
