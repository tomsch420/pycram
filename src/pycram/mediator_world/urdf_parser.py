from typing_extensions import Optional, List

from .geometry import Shape, Box
from .pose import Vector3, Quaternion, Pose, Header, PoseStamped
from .mediator_world import World, Link, Joint
from urdf_parser_py import urdf

from ..datastructures.dataclasses import Color
from ..utils import suppress_stdout_stderr

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
        print(*links, sep="\n")

        return world

    def visual_of_link(self, link: urdf.Link) -> List[Shape]:
        if link.visuals:
            return [self.parse_shape(visual, link) for visual in link.visuals]
        else:
            return []

    def collision_of_link(self, link: urdf.Link) -> List[Shape]:
        ...


    def parse_shape(self, shape: urdf.Visual, link: urdf.Link) -> Shape:
        geometry: urdf.GeometricType = shape.geometry

        if isinstance(geometry, urdf.Box):
            return self.parse_box(geometry, shape, link)

        raise NotImplementedError(f"Parsing of {geometry} not implemented yet.")

    def parse_box(self, box: urdf.Box, shape: urdf.Visual, link: urdf.Link) -> Box:
        pose = self.as_pose_stamped(self.urdf_pose_to_pose(shape.origin), link)

        if shape.material and shape.material.color.rgba:
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
                    visual=self.visual_of_link(link))

    def urdf_pose_to_pose(self, pose: urdf.Pose) -> Pose:
        if pose:
            return Pose(Vector3(*pose.xyz), Quaternion(*pose.rpy, 1.))
        else:
            return Pose()

    def as_pose_stamped(self, pose: Pose, link: Link):
        return PoseStamped(pose=pose, header=Header(frame=link.name))