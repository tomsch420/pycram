"""Module holding all enums of PyCRAM."""

from enum import Enum, auto

from ..failures import UnsupportedJointType


class ExecutionType(Enum):
    """Enum for Execution Process Module types."""
    REAL = auto()
    SIMULATED = auto()
    SEMI_REAL = auto()


class Arms(int, Enum):
    """Enum for Arms."""
    LEFT = 0
    RIGHT = 1
    BOTH = 2


class TaskStatus(int, Enum):
    """
    Enum for readable descriptions of a tasks' status.
    """
    CREATED = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED = 3


class JointType(Enum):
    """
    Enum for readable joint types.
    """
    REVOLUTE = 0
    PRISMATIC = 1
    SPHERICAL = 2
    PLANAR = 3
    FIXED = 4
    UNKNOWN = 5
    CONTINUOUS = 6
    FLOATING = 7


class Grasp(int, Enum):
    """
    Enum for Grasp orientations.
    """
    FRONT = 0
    LEFT = 1
    RIGHT = 2
    TOP = 3
    BACK = 4
    BOTTOM = 5


class ObjectType(int, Enum):
    """
    Enum for Object types to easier identify different objects
    """
    METALMUG = auto()
    PRINGLES = auto()
    MILK = auto()
    SPOON = auto()
    BOWL = auto()
    BREAKFAST_CEREAL = auto()
    JEROEN_CUP = auto()
    ROBOT = auto()
    GRIPPER = auto()
    ENVIRONMENT = auto()
    GENERIC_OBJECT = auto()
    HUMAN = auto()
    IMAGINED_SURFACE = auto()


class State(int, Enum):
    """
    Enumeration which describes the result of a language expression.
    """
    SUCCEEDED = 1
    FAILED = 0
    RUNNING = 2
    INTERRUPTED = 3


class Shape(Enum):
    """
    Enum for visual shapes of objects
    """
    SPHERE = 2
    BOX = 3
    CYLINDER = 4
    MESH = 5
    PLANE = 6
    CAPSULE = 7


class TorsoState(Enum):
    """
    Enum for the different states of the torso.
    """
    HIGH = auto()
    MID = auto()
    LOW = auto()


class WorldMode(Enum):
    """
    Enum for the different modes of the world.
    """
    GUI = "GUI"
    DIRECT = "DIRECT"


class AxisIdentifier(Enum):
    """
    Enum for translating the axis name to a vector along that axis.
    """
    X = (1, 0, 0)
    Y = (0, 1, 0)
    Z = (0, 0, 1)


class GripperState(Enum):
    """
    Enum for the different motions of the gripper.
    """
    OPEN = auto()
    CLOSE = auto()


class GripperType(Enum):
    """
    Enum for the different types of grippers.
    """
    PARALLEL = auto()
    SUCTION = auto()
    FINGER = auto()
    HYDRAULIC = auto()
    PNEUMATIC = auto()
    CUSTOM = auto()


class ImageEnum(Enum):
    """
    Enum for image switch view on hsrb display.
    """
    HI = 0
    TALK = 1
    DISH = 2
    DONE = 3
    DROP = 4
    HANDOVER = 5
    ORDER = 6
    PICKING = 7
    PLACING = 8
    REPEAT = 9
    SEARCH = 10
    WAVING = 11
    FOLLOWING = 12
    DRIVINGBACK = 13
    PUSHBUTTONS = 14
    FOLLOWSTOP = 15
    JREPEAT = 16
    SOFA = 17
    INSPECT = 18
    CHAIR = 37


class DetectionTechnique(int, Enum):
    """
    Enum for techniques for detection tasks.
    """
    ALL = 0
    HUMAN = 1
    TYPES = 2
    REGION = 3
    HUMAN_ATTRIBUTES = 4
    HUMAN_WAVING = 5


class DetectionState(int, Enum):
    """
    Enum for the state of the detection task.
    """
    START = 0
    STOP = 1
    PAUSE = 2


class LoggerLevel(Enum):
    """
    Enum for the different logger levels.
    """
    DEBUG = 'debug'
    INFO = 'info'
    WARN = 'warn'
    ERROR = 'error'
    FATAL = 'fatal'


class VirtualMobileBaseJointName(Enum):
    """
    Enum for the joint names of the virtual mobile base.
    """
    LINEAR_X = "odom_vel_lin_x_joint"
    LINEAR_Y = "odom_vel_lin_y_joint"
    ANGULAR_Z = "odom_vel_ang_z_joint"


class MJCFGeomType(Enum):
    """
    Enum for the different geom types in a MuJoCo XML file.
    """
    BOX = "box"
    CYLINDER = "cylinder"
    CAPSULE = "capsule"
    SPHERE = "sphere"
    PLANE = "plane"
    MESH = "mesh"
    ELLIPSOID = "ellipsoid"
    HFIELD = "hfield"
    SDF = "sdf"


MJCFBodyType = MJCFGeomType
"""
Alias for MJCFGeomType. As the body type is the same as the geom type.
"""


class MJCFJointType(Enum):
    """
    Enum for the different joint types in a MuJoCo XML file.
    """
    FREE = "free"
    BALL = "ball"
    SLIDE = "slide"
    HINGE = "hinge"
    FIXED = "fixed"  # Added for compatibility with PyCRAM, but not a real joint type in MuJoCo.


class MovementType(Enum):
    """
    Enum for the different movement types of the robot.
    """
    STRAIGHT_TRANSLATION = auto()
    STRAIGHT_CARTESIAN = auto()
    TRANSLATION = auto()
    CARTESIAN = auto()


class MultiverseAPIName(Enum):
    """
    Enum for the different APIs of the Multiverse.
    """
    GET_CONTACT_POINTS = "get_contact_points"
    GET_CONTACT_BODIES = "get_contact_bodies"
    GET_CONTACT_BODIES_AND_POINTS = "get_contact_bodies_and_points"
    GET_CONSTRAINT_EFFORT = "get_constraint_effort"
    GET_BOUNDING_BOX = "get_bounding_box"
    ATTACH = "attach"
    DETACH = "detach"
    GET_RAYS = "get_rays"
    EXIST = "exist"
    PAUSE = "pause"
    UNPAUSE = "unpause"
    SAVE = "save"
    LOAD = "load"


class MultiverseProperty(Enum):
    def __str__(self):
        return self.value


class MultiverseBodyProperty(MultiverseProperty):
    """
    Enum for the different properties of a body the Multiverse.
    """
    POSITION = "position"
    ORIENTATION = "quaternion"
    RELATIVE_VELOCITY = "relative_velocity"


class MultiverseJointProperty(MultiverseProperty):
    pass


class MultiverseJointPosition(MultiverseJointProperty):
    """
    Enum for the Position names of the different joint types in the Multiverse.
    """
    REVOLUTE_JOINT_POSITION = "joint_rvalue"
    PRISMATIC_JOINT_POSITION = "joint_tvalue"

    @classmethod
    def from_pycram_joint_type(cls, joint_type: JointType) -> 'MultiverseJointPosition':
        if joint_type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
            return MultiverseJointPosition.REVOLUTE_JOINT_POSITION
        elif joint_type == JointType.PRISMATIC:
            return MultiverseJointPosition.PRISMATIC_JOINT_POSITION
        else:
            raise UnsupportedJointType(joint_type)


class MultiverseJointCMD(MultiverseJointProperty):
    """
    Enum for the Command names of the different joint types in the Multiverse.
    """
    REVOLUTE_JOINT_CMD = "cmd_joint_rvalue"
    PRISMATIC_JOINT_CMD = "cmd_joint_tvalue"

    @classmethod
    def from_pycram_joint_type(cls, joint_type: JointType) -> 'MultiverseJointCMD':
        if joint_type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
            return MultiverseJointCMD.REVOLUTE_JOINT_CMD
        elif joint_type == JointType.PRISMATIC:
            return MultiverseJointCMD.PRISMATIC_JOINT_CMD
        else:
            raise UnsupportedJointType(joint_type)


class FilterConfig(Enum):
    """
    Declare existing filter methods.
    Currently supported: Butterworth
    """
    butterworth = 1
