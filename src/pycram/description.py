from __future__ import annotations

import logging
import os
import pathlib
from abc import ABC, abstractmethod

import trimesh
from geometry_msgs.msg import Point
from trimesh.parent import Geometry3D
from typing_extensions import Tuple, Union, Any, List, Optional, Dict, TYPE_CHECKING, Self, Sequence

from .datastructures.dataclasses import JointState, AxisAlignedBoundingBox, Color, LinkState, VisualShape, \
    MeshVisualShape, RotatedBoundingBox
from .datastructures.enums import JointType
from .datastructures.pose import Pose, Transform
from .datastructures.world_entity import WorldEntity, PhysicalBody
from .failures import ObjectDescriptionNotFound, LinkHasNoGeometry, LinkGeometryHasNoMesh
from .local_transformer import LocalTransformer
from .ros.logging import logwarn_once

if TYPE_CHECKING:
    from .world_concepts.world_object import Object


class EntityDescription(ABC):
    """
    A description of an entity. This can be a link, joint or object description.
    """

    def __init__(self, parsed_description: Optional[Any] = None):
        """
        :param parsed_description: The parsed description (most likely from a description file) of the entity.
        """
        self.parsed_description = parsed_description

    @property
    @abstractmethod
    def origin(self) -> Pose:
        """
        :return: the origin of this entity.
        """
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """
        :return: the name of this entity.
        """
        pass


class LinkDescription(EntityDescription):
    """
    A link description of an object.
    """

    def __init__(self, parsed_link_description: Any, mesh_dir: Optional[str] = None):
        self.parsed_description = parsed_link_description
        self.mesh_dir = mesh_dir

    @property
    @abstractmethod
    def geometry(self) -> Union[List[VisualShape], VisualShape, None]:
        """
        The geometry type of the collision element of this link.
        """
        pass


class JointDescription(EntityDescription):
    """
    A class that represents the description of a joint.
    """

    def __init__(self, parsed_joint_description: Optional[Any] = None, is_virtual: bool = False):
        """
        :param parsed_joint_description: The parsed description of the joint (e.g. from urdf or mjcf file).
        :param is_virtual: True if the joint is virtual (i.e. not a physically existing joint), False otherwise.
        """
        super().__init__(parsed_joint_description)
        self.is_virtual: Optional[bool] = is_virtual

    @property
    @abstractmethod
    def type(self) -> JointType:
        """
        :return: The type of this joint.
        """
        pass

    @property
    @abstractmethod
    def axis(self) -> Point:
        """
        :return: The axis of this joint, for example the rotation axis for a revolute joint.
        """
        pass

    @property
    @abstractmethod
    def has_limits(self) -> bool:
        """
        :return: True if the joint has limits, False otherwise.
        """
        pass

    @property
    def limits(self) -> Tuple[float, float]:
        """
        :return: The lower and upper limits of this joint.
        """
        lower, upper = self.lower_limit, self.upper_limit
        if lower > upper:
            lower, upper = upper, lower
        return lower, upper

    @property
    @abstractmethod
    def lower_limit(self) -> Union[float, None]:
        """
        :return: The lower limit of this joint, or None if the joint has no limits.
        """
        pass

    @property
    @abstractmethod
    def upper_limit(self) -> Union[float, None]:
        """
        :return: The upper limit of this joint, or None if the joint has no limits.
        """
        pass

    @property
    @abstractmethod
    def parent(self) -> str:
        """
        :return: The name of the parent link of this joint.
        """
        pass

    @property
    @abstractmethod
    def child(self) -> str:
        """
        :return: The name of the child link of this joint.
        """
        pass

    @property
    def damping(self) -> float:
        """
        :return: The damping of this joint.
        """
        raise NotImplementedError

    @property
    def friction(self) -> float:
        """
        :return: The friction of this joint.
        """
        raise NotImplementedError


class ObjectEntity:
    """
    An abstract base class that represents a part of an Object.
    This can be a link or a joint of an Object.
    """

    def __init__(self, obj: Object):
        self.object: Object = obj

    @property
    def object_name(self) -> str:
        """
        The name of the object to which this joint belongs.
        """
        return self.object.name

    @property
    def object_id(self) -> int:
        """
        :return: the id of the object to which this entity belongs.
        """
        return self.object.id


class Link(PhysicalBody, ObjectEntity, LinkDescription, ABC):
    """
    A link of an Object in the World.
    """

    def __init__(self, _id: int, link_description: LinkDescription, obj: Object):
        PhysicalBody.__init__(self, _id, obj.world)
        ObjectEntity.__init__(self, obj)
        LinkDescription.__init__(self, link_description.parsed_description, link_description.mesh_dir)
        self.description = link_description
        self.local_transformer: LocalTransformer = LocalTransformer()
        self.constraint_ids: Dict[Link, int] = {}

    @property
    def parent_entity(self) -> Object:
        """
        :return: The parent of this link, which is the object.
        """
        return self.object

    @property
    def name(self) -> str:
        """
        :return: The name of this link.
        """
        return self.description.name

    def get_axis_aligned_bounding_box(self, transform_to_link_pose: bool = True) -> AxisAlignedBoundingBox:
        """
        :param transform_to_link_pose: If True, return the bounding box transformed to the link pose.
        :return: The axis-aligned bounding box of a link. First try to get it from the simulator, if not,
         then calculate it depending on the type of the link geometry.
        """
        try:
            return self.world.get_link_axis_aligned_bounding_box(self)
        except NotImplementedError:
            bounding_box = self.get_axis_aligned_bounding_box_from_geometry()
            if transform_to_link_pose:
                return bounding_box.get_transformed_box(self.transform)
            else:
                return bounding_box

    def get_rotated_bounding_box(self) -> RotatedBoundingBox:
        """
        :return: The rotated bounding box of a link. First try to get it from the simulator, if not,
         then calculate it depending on the type of the link geometry.
        """
        try:
            return self.world.get_link_rotated_bounding_box(self)
        except NotImplementedError:
            return RotatedBoundingBox.from_axis_aligned_bounding_box(self.get_axis_aligned_bounding_box(),
                                                                     self.transform)

    def get_axis_aligned_bounding_box_from_geometry(self) -> AxisAlignedBoundingBox:
        if isinstance(self.geometry, List):
            all_boxes = [geom.get_axis_aligned_bounding_box(self.get_mesh_path(geom))
                         if isinstance(geom, MeshVisualShape) else geom.get_axis_aligned_bounding_box()
                         for geom in self.geometry
                         ]
            bounding_box = AxisAlignedBoundingBox.from_multiple_bounding_boxes(all_boxes)
        else:
            geom = self.geometry
            bounding_box = geom.get_axis_aligned_bounding_box(self.get_mesh_path(geom)) \
                if isinstance(geom, MeshVisualShape) else geom.get_axis_aligned_bounding_box()
        return bounding_box

    def get_convex_hull(self) -> Geometry3D:
        """
        :return: The convex hull of the link geometry.
        """
        try:
            return self.world.get_body_convex_hull(self)
        except NotImplementedError:
            if isinstance(self.geometry, MeshVisualShape):
                mesh_path = self.get_mesh_path(self.geometry)
                mesh = trimesh.load(mesh_path)
                return trimesh.convex.convex_hull(mesh).apply_transform(self.transform.get_homogeneous_matrix())
            else:
                raise LinkGeometryHasNoMesh(self.name, type(self.geometry).__name__)

    def _plot_convex_hull(self):
        """
        Plot the convex hull of the link geometry.
        """
        hull = self.get_convex_hull()
        hull.show()

    def get_mesh_path(self, geometry: MeshVisualShape) -> str:
        """
        :param geometry: The geometry for which the mesh path should be returned.
        :return: The path of the mesh file of this link if the geometry is a mesh.
        """
        return self.get_mesh_filename(geometry)

    def get_mesh_filename(self, geometry: MeshVisualShape) -> str:
        """
        :return: The mesh file name of this link if the geometry is a mesh, otherwise raise a LinkGeometryHasNoMesh.
        :raises LinkHasNoGeometry: If the link has no geometry.
        :raises LinkGeometryHasNoMesh: If the geometry is not a mesh.
        """
        if geometry is None:
            raise LinkHasNoGeometry(self.name)
        if isinstance(geometry, MeshVisualShape):
            return geometry.file_name
        else:
            raise LinkGeometryHasNoMesh(self.name, type(geometry).__name__)

    def set_object_pose_given_link_pose(self, pose: Pose) -> None:
        """
        Set the pose of this link to the given pose.
        NOTE: This will move the entire object such that the link is at the given pose, it will not consider any joints
        that can allow the link to be at the given pose.

        :param pose: The target pose for this link.
        """
        self.object.set_pose(self.get_object_pose_given_link_pose(pose))

    def get_object_pose_given_link_pose(self, pose):
        """
        Get the object pose given the link pose, which could be a hypothetical link pose to see what would be the object
        pose in that case (assuming that the object itself moved not the joints).

        :param pose: The link pose.
        """
        return (pose.to_transform(self.tf_frame) * self.get_transform_to_root_link()).to_pose()

    def get_pose_given_object_pose(self, pose):
        """
        Get the link pose given the object pose, which could be a hypothetical object pose to see what would be the link
        pose in that case (assuming that the object itself moved not the joints).

        :param pose: The object pose.
        """
        return (pose.to_transform(self.object.tf_frame) * self.get_transform_from_root_link()).to_pose()

    def get_transform_from_root_link(self) -> Transform:
        """
        Return the transformation from the root link of the object to this link.
        """
        return self.get_transform_from_link(self.object.root_link)

    def get_transform_to_root_link(self) -> Transform:
        """
        Return the transformation from this link to the root link of the object.
        """
        return self.get_transform_to_link(self.object.root_link)

    @property
    def current_state(self) -> LinkState:
        return LinkState(self.body_state, self.constraint_ids.copy())

    @current_state.setter
    def current_state(self, link_state: LinkState) -> None:
        if self.current_state != link_state:
            if not self.all_constraint_links_belong_to_same_world(link_state):
                raise ValueError("All constraint links must belong to the same world, since the constraint ids"
                                 "are unique to the world and cannot be transferred between worlds.")
            self.body_state = link_state.body_state
            self.constraint_ids = link_state.constraint_ids

    def all_constraint_links_belong_to_same_world(self, other: LinkState) -> bool:
        """
        Check if all links belong to the same world as the links in the other link state.

        :param other: The state of the other link.
        :return: True if all links belong to the same world, False otherwise.
        """
        return all([link.world == other_link.world for link, other_link in zip(self.constraint_ids.keys(),
                                                                               other.constraint_ids.keys())])

    def add_fixed_constraint_with_link(self, child_link: Self,
                                       child_to_parent_transform: Optional[Transform] = None) -> int:
        """
        Add a fixed constraint between this link and the given link, to create attachments for example.

        :param child_link: The child link to which a fixed constraint should be added.
        :param child_to_parent_transform: The transformation between the two links.
        :return: The unique id of the constraint.
        """
        if child_to_parent_transform is None:
            child_to_parent_transform = child_link.get_transform_to_link(self)
        constraint_id = self.world.add_fixed_constraint(self, child_link, child_to_parent_transform)
        self.constraint_ids[child_link] = constraint_id
        child_link.constraint_ids[self] = constraint_id
        return constraint_id

    def remove_constraint_with_link(self, child_link: 'Link') -> None:
        """
        Remove the constraint between this link and the given link.

        :param child_link: The child link of the constraint that should be removed.
        """
        self.world.remove_constraint(self.constraint_ids[child_link])
        del self.constraint_ids[child_link]
        if self in child_link.constraint_ids.keys():
            del child_link.constraint_ids[self]

    @property
    def is_only_link(self) -> bool:
        """
        :return: True if this link is the only link, False otherwise.
        """
        return self.object.has_one_link

    @property
    def is_root(self) -> bool:
        """
        :return: True if this link is the root link, False otherwise.
        """
        return self.object.get_root_link_id() == self.id

    def get_transform_to_link(self, link: 'Link') -> Transform:
        """
        :param link: The link to which the transformation should be returned.
        :return: A Transform object with the transformation from this link to the given link.
        """
        return link.get_transform_from_link(self)

    def get_transform_from_link(self, link: 'Link') -> Transform:
        """
        :param link: The link from which the transformation should be returned.
        :return: A Transform object with the transformation from the given link to this link.
        """
        return self.get_pose_wrt_link(link).to_transform(self.tf_frame)

    def get_pose_wrt_link(self, link: 'Link') -> Pose:
        """
        :param link: The link with respect to which the pose should be returned.
        :return: A Pose object with the pose of this link with respect to the given link.
        """
        return self.local_transformer.transform_pose(self.pose, link.tf_frame)

    def get_origin_transform(self) -> Transform:
        """
        :return: the transformation between the link frame and the origin frame of this link.
        """
        return self.origin.to_transform(self.tf_frame)

    @property
    def pose(self) -> Pose:
        """
        :return: The pose of this link.
        """
        return self.world.get_link_pose(self)

    @pose.setter
    def pose(self, pose: Pose) -> None:
        logwarn_once("Setting the pose of a link is not allowed,"
                     " change object pose and/or joint position to affect the link pose.")

    @property
    def color(self) -> Color:
        """
        :return: A Color object containing the rgba_color of this link.
        """
        return self.world.get_link_color(self)

    @color.setter
    def color(self, color: Color) -> None:
        """
        Set the color of this link, could be rgb or rgba.

        :param color: The color as a list of floats, either rgb or rgba.
        """
        self.world.set_link_color(self, color)

    @property
    def tf_frame(self) -> str:
        """
        The name of the tf frame of this link.
        """
        return f"{self.object.tf_frame}/{self.name}"

    @property
    def origin_transform(self) -> Transform:
        """
        The transformation between the link frame and the origin frame of this link.
        """
        return self.origin.to_transform(self.tf_frame)

    def __copy__(self):
        return Link(self.id, self.description, self.object)


class RootLink(Link, ABC):
    """
    The root link of an Object in the World.
    This differs from the normal AbstractLink class in that the pose and the tf_frame is the same as that of the object.
    """

    def __init__(self, obj: Object):
        Link.__init__(self, obj.get_root_link_id(), obj.get_root_link_description(), obj)

    @property
    def tf_frame(self) -> str:
        """
        :return: the tf frame of the root link, which is the same as the tf frame of the object.
        """
        return self.object.tf_frame

    @property
    def pose(self) -> Pose:
        """
        :return: The pose of the root link, which is the same as the pose of the object.
        """
        return self.object.pose

    def __copy__(self):
        return RootLink(self.object)


class Joint(WorldEntity, ObjectEntity, JointDescription, ABC):
    """
    Represent a joint of an Object in the World.
    """

    def __init__(self, _id: int,
                 joint_description: JointDescription,
                 obj: Object, is_virtual: Optional[bool] = False):
        WorldEntity.__init__(self, _id, obj.world)
        ObjectEntity.__init__(self, obj)
        JointDescription.__init__(self, joint_description.parsed_description, is_virtual)
        self.description = joint_description
        self.acceptable_error = (self.world.conf.revolute_joint_position_tolerance if self.type == JointType.REVOLUTE
                                 else self.world.conf.prismatic_joint_position_tolerance)
        self._update_position()

    @property
    def name(self) -> str:
        """
        :return: The name of this joint.
        """
        return self.description.name

    @property
    def parent_entity(self) -> Link:
        """
        :return: The parent of this joint, which is the object.
        """
        return self.parent_link

    @property
    def tf_frame(self) -> str:
        """
        The tf frame of a joint is the tf frame of the child link.
        """
        return self.child_link.tf_frame

    @property
    def pose(self) -> Pose:
        """
        :return: The pose of this joint. The pose is the pose of the child link of this joint.
        """
        return self.child_link.pose

    def _update_position(self) -> None:
        """
        Update the current position of the joint from the physics simulator.
        """
        self._current_position = self.world.get_joint_position(self)

    @property
    def parent_link(self) -> Link:
        """
        :return: The parent link as a AbstractLink object.
        """
        return self.object.get_link(self.parent)

    @property
    def child_link(self) -> Link:
        """
        :return: The child link as a AbstractLink object.
        """
        return self.object.get_link(self.child)

    @property
    def position(self) -> float:
        if self.world.conf.update_poses_from_sim_on_get:
            self._update_position()
        return self._current_position

    def reset_position(self, position: float) -> None:
        self.world.reset_joint_position(self, position)
        self._update_position()

    def get_object_id(self) -> int:
        """
        :return: The integer id of the object to which this joint belongs.
        """
        return self.object.id

    @position.setter
    def position(self, joint_position: float) -> None:
        """
        Set the position of the given joint to the given joint pose. If the pose is outside the joint limits,
         issue a warning. However, set the joint either way.

        :param joint_position: The target pose for this joint
        """
        # TODO Limits for rotational (infinite) joints are 0 and 1, they should be considered separately
        if self.has_limits:
            low_lim, up_lim = self.limits
            if not low_lim <= joint_position <= up_lim:
                logging.warning(
                    f"The joint position has to be within the limits of the joint. The joint limits for {self.name}"
                    f" are {low_lim} and {up_lim}")
                logging.warning(f"The given joint position was: {joint_position}")
                # Temporarily disabled because kdl outputs values exciting joint limits
                # return
        self.reset_position(joint_position)

    def enable_force_torque_sensor(self) -> None:
        self.world.enable_joint_force_torque_sensor(self.object, self.id)

    def disable_force_torque_sensor(self) -> None:
        self.world.disable_joint_force_torque_sensor(self.object, self.id)

    def get_reaction_force_torque(self) -> List[float]:
        return self.world.get_joint_reaction_force_torque(self.object, self.id)

    def get_applied_motor_torque(self) -> float:
        return self.world.get_applied_joint_motor_torque(self.object, self.id)

    @property
    def current_state(self) -> JointState:
        return JointState(self.position, self.acceptable_error)

    @current_state.setter
    def current_state(self, joint_state: JointState) -> None:
        """
        Update the current state of this joint from the given joint state if the position is different.

        :param joint_state: The joint state to update from.
        """
        if self.current_state != joint_state:
            self.position = joint_state.position

    def __copy__(self):
        return Joint(self.id, self.description, self.object, self.is_virtual)


class ObjectDescription(EntityDescription):
    """
    A class that represents the description of an object.
    """

    mesh_extensions: Tuple[str] = (".obj", ".stl", ".dae", ".ply")
    """
    The file extensions of the mesh files that can be used to generate a description file.
    """

    class Link(Link, ABC):
        ...

    class RootLink(RootLink, ABC):
        ...

    class Joint(Joint, ABC):
        ...

    def __init__(self, path: Optional[str] = None):
        """
        :param path: The path of the file to update the description data from.
        """
        super().__init__(None)
        self._links: Optional[List[LinkDescription]] = None
        self._joints: Optional[List[JointDescription]] = None
        self._link_map: Optional[Dict[str, Any]] = None
        self._joint_map: Optional[Dict[str, Any]] = None
        self.original_path: Optional[str] = path

        if path:
            self.update_description_from_file(path)
        else:
            self._parsed_description = None

        self.virtual_joint_names: List[str] = []

    @property
    @abstractmethod
    def child_map(self) -> Dict[str, List[Tuple[str, str]]]:
        """
        :return: A dictionary mapping the name of a link to its children which are represented as a tuple of the child
            joint name and the link name.
        """
        pass

    @property
    @abstractmethod
    def parent_map(self) -> Dict[str, Tuple[str, str]]:
        """
        :return: A dictionary mapping the name of a link to its parent joint and link as a tuple.
        """
        pass

    @property
    @abstractmethod
    def link_map(self) -> Dict[str, LinkDescription]:
        """
        :return: A dictionary mapping the name of a link to its description.
        """
        pass

    @property
    @abstractmethod
    def joint_map(self) -> Dict[str, JointDescription]:
        """
        :return: A dictionary mapping the name of a joint to its description.
        """
        pass

    def is_joint_virtual(self, name: str) -> bool:
        """
        :param name: The name of the joint.
        :return: True if the joint is virtual, False otherwise.
        """
        return name in self.virtual_joint_names

    @abstractmethod
    def add_joint(self, name: str, child: str, joint_type: JointType,
                  axis: Point, parent: Optional[str] = None, origin: Optional[Pose] = None,
                  lower_limit: Optional[float] = None, upper_limit: Optional[float] = None,
                  is_virtual: Optional[bool] = False) -> None:
        """
        Add a joint to this object.

        :param name: The name of the joint.
        :param child: The name of the child link.
        :param joint_type: The type of the joint.
        :param axis: The axis of the joint.
        :param parent: The name of the parent link.
        :param origin: The origin of the joint.
        :param lower_limit: The lower limit of the joint.
        :param upper_limit: The upper limit of the joint.
        :param is_virtual: True if the joint is virtual, False otherwise.
        """
        pass

    def update_description_from_file(self, path: str) -> None:
        """
        Update the description of this object from the file at the given path.

        :param path: The path of the file to update from.
        """
        self._parsed_description = self.load_description(path)

    def update_description_from_string(self, description_string: str) -> None:
        """
        Update the description of this object from the given description string.

        :param description_string: The description string to update from.
        """
        self._parsed_description = self.load_description_from_string(description_string)

    def load_description_from_string(self, description_string: str) -> Any:
        """
        Load the description from the given string.

        :param description_string: The description string to load from.
        """
        raise NotImplementedError

    @property
    def parsed_description(self) -> Any:
        """
        :return: The object parsed from the description file.
        """
        return self._parsed_description

    @parsed_description.setter
    def parsed_description(self, parsed_description: Any):
        """
        :param parsed_description: The parsed description object (depends on the description file type).
        """
        self._parsed_description = parsed_description

    @abstractmethod
    def load_description(self, path: str) -> Any:
        """
        Load the description from the file at the given path.

        :param path: The path to the source file, if only a filename is provided then the resources directories will be
         searched.
        """
        pass

    def generate_description_from_file(self, path: str, name: str, extension: str, save_path: str,
                                       scale_mesh: Optional[float] = None,
                                       mesh_transform: Optional[Transform] = None,
                                       color: Optional[Color] = None) -> None:
        """
        Generate and preprocess the description from the file at the given path and save the preprocessed
        description. The generated description will be saved at the given save path.

        :param path: The path of the file to preprocess.
        :param name: The name of the object.
        :param extension: The file extension of the file to preprocess.
        :param save_path: The path to save the generated description file.
        :param scale_mesh: The scale of the mesh.
        :param mesh_transform: The transformation matrix to apply to the mesh.
        :param color: The color of the object.
        :raises ObjectDescriptionNotFound: If the description file could not be found/read.
        """

        if extension in self.mesh_extensions:
            if extension == ".ply":
                mesh = trimesh.load(path)
                if scale_mesh is not None:
                    mesh.apply_scale(scale_mesh)
                if mesh_transform is not None:
                    transform = mesh_transform.get_homogeneous_matrix()
                    mesh.apply_transform(transform)
                path = path.replace(extension, ".obj")
                mesh.export(path)
            self.generate_from_mesh_file(path, name, save_path=save_path, color=color, scale=scale_mesh)
        elif extension == self.get_file_extension():
            self.generate_from_description_file(path, save_path=save_path)
        else:
            try:
                # Using the description from the parameter server
                self.generate_from_parameter_server(path, save_path=save_path)
            except KeyError:
                logging.warning(f"Couldn't find file data in the ROS parameter server")

        if not self.check_description_file_exists_and_can_be_read(save_path):
            raise ObjectDescriptionNotFound(name, path, extension)

    @staticmethod
    def check_description_file_exists_and_can_be_read(path: str) -> bool:
        """
        Check if the description file exists at the given path.

        :param path: The path to the description file.
        :return: True if the file exists, False otherwise.
        """
        exists = os.path.exists(path)
        if exists:
            with open(path, "r") as file:
                exists = bool(file.read())
        return exists

    @staticmethod
    def write_description_to_file(description_string: str, save_path: str) -> None:
        """
        Write the description string to the file at the given path.

        :param description_string: The description string to write.
        :param save_path: The path of the file to write to.
        """
        with open(save_path, "w") as file:
            file.write(description_string)

    def get_file_name(self, path_object: pathlib.Path, extension: str, object_name: str) -> str:
        """
        :param path_object: The path object of the description file or the mesh file.
        :param extension: The file extension of the description file or the mesh file.
        :param object_name: The name of the object.
        :return: The file name of the description file.
        """
        if extension in self.mesh_extensions:
            file_name = path_object.stem + self.get_file_extension()
        elif extension == self.get_file_extension():
            file_name = path_object.name
        else:
            file_name = object_name + self.get_file_extension()

        return file_name

    @classmethod
    @abstractmethod
    def generate_from_mesh_file(cls, path: str, name: str, save_path: str, color: Color) -> None:
        """
        Generate a description file from one of the mesh types defined in the mesh_extensions and
        return the path of the generated file. The generated file will be saved at the given save_path.

        :param path: The path to the .obj file.
        :param name: The name of the object.
        :param save_path: The path to save the generated description file.
        :param color: The color of the object.
        """
        pass

    @classmethod
    @abstractmethod
    def generate_from_description_file(cls, path: str, save_path: str, make_mesh_paths_absolute: bool = True) -> None:
        """
        Preprocess the given file and return the preprocessed description string. The preprocessed description will be
        saved at the given save_path.

        :param path: The path of the file to preprocess.
        :param save_path: The path to save the preprocessed description file.
        :param make_mesh_paths_absolute: Whether to make the mesh paths absolute.
        """
        pass

    @classmethod
    @abstractmethod
    def generate_from_parameter_server(cls, name: str, save_path: str) -> None:
        """
        Preprocess the description from the ROS parameter server and return the preprocessed description string.
        The preprocessed description will be saved at the given save_path.

        :param name: The name of the description on the parameter server.
        :param save_path: The path to save the preprocessed description file.
        """
        pass

    @property
    @abstractmethod
    def links(self) -> List[LinkDescription]:
        """
        :return: A list of links descriptions of this object.
        """
        pass

    def get_link_by_name(self, link_name: str) -> LinkDescription:
        """
        :return: The link description with the given name.
        """
        return self.link_map[link_name]

    @property
    @abstractmethod
    def joints(self) -> List[JointDescription]:
        """
        :return: A list of joints descriptions of this object.
        """
        pass

    def get_joint_by_name(self, joint_name: str) -> JointDescription:
        """
        :return: The joint description with the given name.
        """
        return self.joint_map[joint_name]

    @abstractmethod
    def get_root(self) -> str:
        """
        :return: the name of the root link of this object.
        """
        pass

    def get_tip(self) -> str:
        """
        :return: the name of the tip link of this object.
        """
        raise NotImplementedError

    @abstractmethod
    def get_chain(self, start_link_name: str, end_link_name: str, joints: Optional[bool] = True,
                  links: Optional[bool] = True, fixed: Optional[bool] = True) -> List[str]:
        """
        :return: the chain of links from 'start_link_name' to 'end_link_name'.
        """
        pass

    @staticmethod
    @abstractmethod
    def get_file_extension() -> str:
        """
        :return: The file extension of the description file.
        """
        pass
