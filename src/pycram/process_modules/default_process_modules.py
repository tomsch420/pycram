import inspect

import numpy as np
import rospy
from typing_extensions import List, TYPE_CHECKING

from pycrap import *
from ..datastructures.enums import JointType
from ..datastructures.world import World
from ..designators.motion_designator import *
from ..external_interfaces import giskard
from ..external_interfaces.ik import request_ik
from ..external_interfaces.move_base import query_pose_nav
from ..external_interfaces.robokudo import query_all_objects, query_object, query_human, query_specific_region, \
    query_human_attributes, query_waving_human
from ..failures import NavigationGoalNotReachedError
from ..local_transformer import LocalTransformer
from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription
from ..process_module import ProcessModule
from ..robot_description import RobotDescription
from ..ros.data_types import Duration
from ..ros.logging import logdebug, loginfo, logwarn
from ..ros.ros_tools import get_time
from ..utils import _apply_ik, map_color_names_to_rgba
from ..world_concepts.world_object import Object
from ..world_reasoning import visible, link_pose_for_joint_config

if TYPE_CHECKING:
    from ..designators.object_designator import ObjectDesignatorDescription


class DefaultNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = World.robot
        robot.set_pose(desig.target)


class DefaultMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position("head_pan_joint")
        current_tilt = robot.get_joint_position("head_tilt_joint")

        robot.set_joint_position("head_pan_joint", new_pan + current_pan)
        robot.set_joint_position("head_tilt_joint", new_tilt + current_tilt)


class DefaultMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only move one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        robot_description = RobotDescription.current_robot_description
        gripper = desig.gripper
        arm_chain = robot_description.get_arm_chain(gripper)
        if arm_chain.end_effector.gripper_object_name is not None:
            robot = World.current_world.get_object_by_name(arm_chain.end_effector.gripper_object_name)
        else:
            robot = World.robot
        motion = desig.motion
        robot.set_multiple_joint_positions(arm_chain.get_static_gripper_state(motion))


class DefaultDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    :return: A list of perceived objects.
    """

    def _execute(self, designator: DetectingMotion):
        robot = World.robot
        cam_link_name = RobotDescription.current_robot_description.get_camera_link()
        camera_description = RobotDescription.current_robot_description.cameras[
            list(RobotDescription.current_robot_description.cameras.keys())[0]]
        front_facing_axis = camera_description.front_facing_axis
        query_result = []
        world_objects = []
        try:
            object_types = designator.object_designator_description.types
        except AttributeError:
            object_types = None
        if designator.technique == DetectionTechnique.TYPES:
            for obj_type in object_types:
                list1 = World.current_world.get_object_by_type(obj_type)
                world_objects = world_objects + list1
        elif designator.technique == DetectionTechnique.ALL:
            world_objects = World.current_world.get_scene_objects()
        elif designator.technique == DetectionTechnique.HUMAN:
            raise NotImplementedError("Detection by human is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.REGION:
            raise NotImplementedError("Detection by region is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.HUMAN_ATTRIBUTES:
            raise NotImplementedError("Detection by human attributes is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.HUMAN_WAVING:
            raise NotImplementedError("Detection by waving human is not yet implemented in simulation")
        for obj in world_objects:
            if visible(obj, robot.get_link_pose(cam_link_name), front_facing_axis):
                query_result.append(obj)
        if query_result is None:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {object_types} in the FOV of the robot")
        else:
            object_dict = []

            for obj in query_result:
                object_dict.append(ObjectDesignatorDescription.Object(obj.name, obj.obj_type,
                                                   obj))

            return object_dict


class DefaultMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        target = desig.target
        robot = World.robot

        _move_arm_tcp(target, robot, desig.arm)


class DefaultMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):

        robot = World.robot
        if desig.right_arm_poses:
            for joint, pose in desig.right_arm_poses.items():
                robot.set_joint_position(joint, pose)
        if desig.left_arm_poses:
            for joint, pose in desig.left_arm_poses.items():
                robot.set_joint_position(joint, pose)


class DefaultMoveJoints(ProcessModule):
    def _execute(self, desig: MoveJointsMotion):
        robot = World.robot
        for joint, pose in zip(desig.names, desig.positions):
            robot.set_joint_position(joint, pose)


class DefaultWorldStateDetecting(ProcessModule):
    """
    This process moduledetectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.obj_type == obj_type, World.current_world.objects))[0]


class DefaultOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[1])


class DefaultClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


###########################################################
########## Process Modules for the Real     ###############
###########################################################

class DefaultDetectingReal(ProcessModule):
    def _execute(self, designator: DetectingMotion) -> List[Object]:
        """
            Perform a query based on the detection technique and state defined in the designator.

            :return: A list of perceived objects.
            """
        object_designator_description = designator.object_designator_description
        query_methods = {
            DetectionTechnique.TYPES: lambda: query_object(object_designator_description),
            DetectionTechnique.HUMAN: lambda: query_human(),
            DetectionTechnique.HUMAN_ATTRIBUTES: query_human_attributes,
            DetectionTechnique.HUMAN_WAVING: query_waving_human,
            DetectionTechnique.REGION: lambda: query_specific_region(designator.region)
        }  # Fetch the appropriate query function
        query_func = query_methods.get(designator.technique, query_all_objects)
        query_result = query_func() if callable(query_func) else query_func
        # Handle the case where no result is found
        if query_result is None:
            raise PerceptionObjectNotFound(
                f"Could not find an object in the FOV of the robot")
        else:
            perceived_objects = []
            for i in range(0, len(query_result.res)):
                try:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
                except IndexError:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose)
                    pass
                obj_type = query_result.res[i].type
                obj_size = None
                try:
                    obj_size = query_result.res[i].shape_size[0].dimensions
                except IndexError:
                    pass
                obj_color = None
                try:
                    obj_color = query_result.res[i].color[0]
                except IndexError:
                    pass

                hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]

                # Check if the object type is a subclass of the classes in the objects module (pycrap)
                class_names = [name for name, obj in inspect.getmembers(objects, inspect.isclass)]

                matching_classes = [class_name for class_name in class_names if obj_type in class_name]

                obj_name = obj_type + "" + str(get_time())
                # Check if there are any matches
                if matching_classes:
                    rospy.loginfo(f"Matching class names: {matching_classes}")
                    obj_type = matching_classes[0]
                else:
                    rospy.loginfo(f"No class name contains the string '{obj_type}'")
                    obj_type = Genobj
                gen_obj_desc = GenericObjectDescription(obj_name, [0, 0, 0], hsize)
                color = map_color_names_to_rgba(obj_color)
                generic_obj = Object(name=obj_name, concept=obj_type, path=None, description=gen_obj_desc, color=color)

                generic_obj.set_pose(obj_pose)

                perceived_objects.append(generic_obj)

            object_dict = []

            for obj in perceived_objects:
                object_dict.append(ObjectDesignatorDescription.Object(obj.name, obj.obj_type,
                                                                      obj))

            return object_dict

class DefaultNavigationReal(ProcessModule):
    """
    Process module for the real robot that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion):
        logdebug(f"Sending goal to movebase to Move the robot")
        query_pose_nav(designator.target)
        if not World.current_world.robot.pose.almost_equal(designator.target, 0.05, 3):
            raise NavigationGoalNotReachedError(World.current_world.robot.pose, designator.target)

class DefaultMoveHeadReal(ProcessModule):
    """
    Process module for controlling the real robot's head to look at a specified position.
    Uses the same calculations as the simulated version to orient the head.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, np.sqrt(pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2)) * -1

        current_pan = robot.get_joint_position("head_pan_joint")
        current_tilt = robot.get_joint_position("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_pan_joint": new_pan + current_pan,
                                    "head_tilt_joint": new_tilt + current_tilt})


class DefaultMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real robot while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion):
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        tip_link = RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame()
        root_link = "map"

        gripper_that_can_collide = designator.arm if designator.allow_gripper_collision else None
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm.name.lower())

        if designator.movement_type == MovementType.STRAIGHT_TRANSLATION:
            giskard.achieve_straight_translation_goal(pose_in_map.position_as_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.STRAIGHT_CARTESIAN:
            giskard.achieve_straight_cartesian_goal(pose_in_map, tip_link, root_link)
        elif designator.movement_type == MovementType.TRANSLATION:
            giskard.achieve_translation_goal(pose_in_map.position_as_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.CARTESIAN:
            giskard.achieve_cartesian_goal(pose_in_map, tip_link, root_link,
                                           grippers_that_can_collide=gripper_that_can_collide,
                                           use_monitor=designator.monitor_motion)
        if not World.current_world.robot.get_link_pose(tip_link).almost_equal(designator.target, 0.01, 3):
            raise ToolPoseNotReachedError(World.current_world.robot.get_link_pose(tip_link), designator.target)



class DefaultMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real robot to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion):
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class DefaultMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion):
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class DefaultMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real robot, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion):
        raise NotImplementedError(f"There is DefaultMoveGripperReal process module")


class DefaultOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion):
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion):
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("default")

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if (ProcessModuleManager.execution_type == ExecutionType.SIMULATED or
                ProcessModuleManager.execution_type == ExecutionType.REAL):
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultCloseReal(self._close_lock)
