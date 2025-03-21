from __future__ import annotations

import enum
import os
from dataclasses import dataclass, field

from typing_extensions import Optional, List, Self

from .mediator_world import Link, LinkView
from .urdf_parser import URDFParser
from ..datastructures.enums import AxisIdentifier
from ..ros import get_ros_package_path


@dataclass
class RobotLink(Link):
    _robot: Optional[AbstractRobot] = field(default=None, init=False)


@dataclass
class RobotLinkView(LinkView):
    _robot: AbstractRobot = field(default=None, init=False)
    start_link: RobotLink
    end_link: Optional[RobotLink] = None

@dataclass
class RobotBase(RobotLinkView):
    torso: RobotLinkView = field(default_factory=RobotLinkView)
    kinematic_chains: List[KinematicChain] = field(default_factory=list)

class WheelBaseType(str, enum.Enum):
    OMNI = "omni"
    DIFFERENTIAL = "differential"
    ACKERMANN = "ackermann"
    MECANUM = "mecanum"
    TRACKS = "tracks"

class Direction(int, enum.Enum):
    FORWARD = 1
    BACKWARD = -1

@dataclass
class AxisDirection:
    axis: AxisIdentifier
    direction: Direction

@dataclass
class WheeledBase(RobotBase):
    type: WheelBaseType = field(default_factory=str)


@dataclass
class Finger(RobotLinkView):
    ...


@dataclass
class Manipulator(RobotLinkView):
    ...


@dataclass
class Gripper(Manipulator):
    fingers: List[Finger] = field(default_factory=list)
    thumb: Optional[Finger] = None

@dataclass
class Sensor(RobotLink):
    ...

@dataclass
class FieldOfView:
    vertical_angle: float
    horizontal_angle: float

@dataclass
class Camera(Sensor):
    forward_facing_axis: AxisDirection = field(default_factory=AxisDirection)
    field_of_view: FieldOfView = field(default_factory=FieldOfView)
    minimal_height: float = 0.0
    maximal_height: float = 1.0

@dataclass
class KinematicChain(RobotLinkView):
    manipulator: Optional[Manipulator] = None
    sensors: Optional[List[Sensor]] = None

@dataclass
class Torso(RobotLinkView):
    kinematic_chains: List[KinematicChain] = field(default_factory=list)

@dataclass
class AbstractRobot(LinkView):
    base: RobotBase
    manipulator_chains: Optional[List[KinematicChain]] = None
    sensor_chains: Optional[List[KinematicChain]] = None
    torso : Optional[Torso] = None


@dataclass
class PR2(AbstractRobot):

    @classmethod
    def make_from_urdf(cls, filename: Optional[str] = None) -> Self:
        if filename is None:
            filename = "resources/robots/pr2.urdf"
            filename = os.path.join(get_ros_package_path("pycram"), filename)

        parser = URDFParser(filename)
        world = parser.parse()
        torso_link = RobotLink.from_link(world.get_link_by_name("torso_lift_link"))

        left_fingers_links = ["l_gripper_l_finger_link", "l_gripper_r_finger_link"]
        left_fingers = [Finger(start_link=RobotLink.from_link(world.get_link_by_name(link))) for link in left_fingers_links]
        left_thumb = Finger(start_link=RobotLink.from_link(world.get_link_by_name("l_gripper_l_finger_link")))
        left_tool_frame = RobotLink.from_link(world.get_link_by_name("l_gripper_tool_frame"))
        left_gripper = Gripper(start_link=RobotLink.from_link(world.get_link_by_name("l_gripper_palm_link")), fingers=left_fingers, thumb=left_thumb, end_link=left_tool_frame)
        left_arm = KinematicChain(start_link=RobotLink.from_link(world.get_link_by_name("l_shoulder_pan_link")),
                                  manipulator=left_gripper)

        right_fingers_links = ["r_gripper_l_finger_link", "r_gripper_r_finger_link"]
        right_fingers = [Finger(start_link=RobotLink.from_link(world.get_link_by_name(link))) for link in right_fingers_links]
        right_thumb = Finger(start_link=RobotLink.from_link(world.get_link_by_name("r_gripper_l_finger_link")))
        right_tool_frame = RobotLink.from_link(world.get_link_by_name("r_gripper_tool_frame"))
        right_gripper = Gripper(start_link=RobotLink.from_link(world.get_link_by_name("r_gripper_palm_link")), fingers=right_fingers, thumb=right_thumb, end_link=right_tool_frame)
        right_arm = KinematicChain(start_link=RobotLink.from_link(world.get_link_by_name("r_shoulder_pan_link")),
                                   manipulator=right_gripper)

        head = KinematicChain(start_link=RobotLink.from_link(world.get_link_by_name("head_pan_link")),
                              end_link=RobotLink.from_link(world.get_link_by_name("head_tilt_link")),
                              sensors=[Camera(RobotLink.from_link(world.get_link_by_name("head_camera_link")))])

        torso = Torso(start_link=torso_link, end_link=torso_link, kinematic_chains=[left_arm, right_arm, head])
        base = WheeledBase(start_link=RobotLink.from_link(world.get_link_by_name("base_link")),
                           type=WheelBaseType.OMNI, torso=torso, kinematic_chains=[left_arm, right_arm, head])

        cls.base = base
        cls.torso = torso
        cls.manipulator_chains = [left_arm, right_arm]
        cls.sensor_chains = [head]