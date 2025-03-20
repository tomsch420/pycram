from typing_extensions import Optional, List, Union

from .geometry import Shape, Box
from .pose import Vector3, Quaternion, Pose, Header, PoseStamped
from .mediator_world import World, Link, Joint, JointAxis
from urdf_parser_py import urdf

from ..datastructures.dataclasses import Color
from ..datastructures.enums import JointType, AxisIdentifier
from ..utils import suppress_stdout_stderr

joint_type_map = {'unknown': JointType.UNKNOWN,
                 'revolute': JointType.REVOLUTE,
                 'continuous': JointType.CONTINUOUS,
                 'prismatic': JointType.PRISMATIC,
                 'floating': JointType.FLOATING,
                 'planar': JointType.PLANAR,
                 'fixed': JointType.FIXED}

class URDFParser:
    """
    Class to parse URDF files to world objects.
    """

    file_path: str

    def __init__(self, file: str):
        self.file_path = file

    def parse(self) -> World:
        world = World()

        with open(self.file_path, 'r') as file:
            # Since parsing URDF causes a lot of warning messages which can't be deactivated, we suppress them
            with suppress_stdout_stderr():
                parsed = urdf.URDF.from_xml_string(file.read())

        links = [self.parse_link(link) for link in parsed.links]
        [world.add_link(link) for link in links]

        for joint in parsed.joints:
            parent = world.get_link_by_name(joint.parent)
            child = world.get_link_by_name(joint.child)
            parsed_joint = self.parse_joint(joint, parent, child)
            world.add_joint(parsed_joint)

        return world

    def parse_joint(self, joint: urdf.Joint, parent: Link, child: Link) -> Joint:
        axis = self.parse_joint_axis(joint.axis)

        lower = None
        upper = None
        if joint.limit:
            lower = joint.limit.lower
            upper = joint.limit.upper

        result = Joint(type=joint_type_map[joint.type], parent=parent, child=child,
                       axis=axis, lower_limit=lower, upper_limit=upper)
        return result

    def parse_joint_axis(self, axis) -> JointAxis:
        result = JointAxis(0)
        if axis:
            if axis.x:
                result |= JointAxis.X
            if axis.y:
                result |= JointAxis.Y
            if axis.z:
                result |= JointAxis.Z
        return result

    def visual_of_link(self, link: urdf.Link) -> List[Shape]:
        if link.visuals:
            return [self.parse_shape(visual, link) for visual in link.visuals]
        else:
            return []

    def collision_of_link(self, link: urdf.Link) -> List[Shape]:
        if link.collisions:
            return [self.parse_shape(collision, link) for collision in link.collisions]
        else:
            return []

    def parse_shape(self, shape: urdf.Visual, link: urdf.Link) -> Shape:
        geometry: urdf.GeometricType = shape.geometry

        if isinstance(geometry, urdf.Box):
            return self.parse_box(geometry, shape, link)

        raise NotImplementedError(f"Parsing of {geometry} not implemented yet.")

    def parse_box(self, box: urdf.Box, shape: Union[urdf.Visual, urdf.Collision], link: urdf.Link) -> Box:
        pose = self.as_pose_stamped(self.urdf_pose_to_pose(shape.origin), link)

        if isinstance(shape, urdf.Visual) and shape.material and shape.material.color.rgba:
            color = Color(shape.material.color.rgba)
        else:
            color = Color()

        result = Box(length=box.size[0], width=box.size[1], height=box.size[2], pose=pose, color=color)
        return result

    def parse_link(self, link: urdf.Link) -> Link:
        """
        Parses a URDF link to a link object.
        :param link: The URDF link to parse.
        :return: The parsed link object.
        """
        return Link(link.name, pose=self.as_pose_stamped(self.urdf_pose_to_pose(link.origin), link),
                    visual=self.visual_of_link(link), collision=self.collision_of_link(link))

    def urdf_pose_to_pose(self, pose: urdf.Pose) -> Pose:
        if pose:
            return Pose(Vector3(*pose.xyz), Quaternion(*pose.rpy, 1.))
        else:
            return Pose()

    def as_pose_stamped(self, pose: Pose, link: Link):
        return PoseStamped(pose=pose, header=Header(frame=link.name))