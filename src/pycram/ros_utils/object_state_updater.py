import atexit

import rospy
import tf
import time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ..datastructures.world import World
from ..robot_description import RobotDescription
from ..datastructures.pose import Pose
from ..ros.data_types import Time, Duration
from ..ros.ros_tools import wait_for_message, create_timer


class RobotStateUpdater:
    """
    Updates the robot in the World with information of the real robot published to ROS topics.
    Infos used to update the robot are:

        * The current pose of the robot
        * The current joint state of the robot
    .. Note:: This class can only be used if the topics are present in the RSO network on the real robot/world,
    hence it is not testable in the CI.
    """

    def __init__(self, tf_topic: str, joint_state_topic: str):
        """
        The robot state updater uses a TF topic and a joint state topic to get the current state of the robot.

        :param tf_topic: Name of the TF topic, needs to publish geometry_msgs/TransformStamped
        :param joint_state_topic: Name of the joint state topic, needs to publish sensor_msgs/JointState
        """
        self.tf_listener = tf.TransformListener()
        self.tf_listener.clear()

        time.sleep(1)
        self.tf_topic = tf_topic
        self.joint_state_topic = joint_state_topic
        self.tf_timer = create_timer(Duration().from_sec(0.1), self._subscribe_tf)
        self.joint_state_timer = create_timer(Duration().from_sec(0.1), self._subscribe_joint_state)

        atexit.register(self._stop_subscription)

    def _subscribe_tf(self, msg: TransformStamped) -> None:
        """
        Callback for the TF timer, will do a lookup of the transform between map frame and the robot base frame.

        :param msg: TransformStamped message published to the topic
        """
        self.tf_listener.waitForTransform("map", RobotDescription.current_robot_description.base_link, Time(0.0), Duration(5))
        trans, rot = self.tf_listener.lookupTransform("map", RobotDescription.current_robot_description.base_link, Time(0.0))
        World.robot.set_pose(Pose(trans, rot))

    def _subscribe_joint_state(self, msg: JointState) -> None:
        """
        Sets the current joint configuration of the robot in the world to the configuration published on the
        topic. Since this uses rospy.wait_for_message which can have errors when used with threads there might be an
        attribute error in the rospy implementation.

        :param msg: JointState message published to the topic.
        """
        try:
            msg = wait_for_message(self.joint_state_topic, JointState)
            for name, position in zip(msg.name, msg.position):
                World.robot.set_joint_position(name, position)
        except AttributeError:
            pass

    def _stop_subscription(self) -> None:
        """
        Stops the Timer for TF and joint states and therefore the updating of the robot in the world.
        """
        self.tf_timer.shutdown()
        self.joint_state_timer.shutdown()


class EnvironmentStateUpdater:
    """
    Updates the environment in the World with information of the real environment published to ROS topics.
    Infos used to update the envi are:
        * The current pose of the environment
        * The current joint state of the environment

    .. Note:: This class can only be used if the topics are present in the RSO network on the real robot/world,
    hence it is not testable in the CI.
    """

    def __init__(self, tf_topic: str, joint_state_topic: str):
        """
        The environment state updater uses a TF topic and a joint state topic to get the current state of the environment.

        :param tf_topic: Name of the TF topic, needs to publish geometry_msgs/TransformStamped
        :param joint_state_topic: Name of the joint state topic, needs to publish sensor_msgs/JointState
        """
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self.tf_topic = tf_topic
        self.joint_state_topic = joint_state_topic

        self.joint_state_timer = rospy.Timer(rospy.Duration(0.1), self._subscribe_joint_state)

        atexit.register(self._stop_subscription)

    def _subscribe_joint_state(self, msg: JointState) -> None:
        """
        Sets the current joint configuration of the environment in the world to the configuration published on the topic.
        Since this uses rospy.wait_for_message which can have errors when used with threads there might be an attribute error
        in the rospy implementation.

        :param msg: JointState message published to the topic.
        """
        try:
            msg = rospy.wait_for_message(self.joint_state_topic, JointState)
            for name, position in zip(msg.name, msg.position):
                try:
                    # Attempt to get the joint state. This might throw a KeyError if the joint name doesn't exist
                    if World.environment.get_joint_state(name) is None:
                        continue
                    # Set the joint state if the joint exists
                    World.environment.set_joint_state(name, position)
                except KeyError:
                    # Handle the case where the joint name does not exist
                    pass
        except AttributeError:
            pass

    def _stop_subscription(self) -> None:
        """
        Stops the Timer for TF and joint states and therefore the updating of the environment in the world.
        """
        self.joint_state_timer.shutdown()
        self.joint_state_timer.shutdown()
