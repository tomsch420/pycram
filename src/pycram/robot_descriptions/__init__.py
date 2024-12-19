from os.path import dirname, basename, isfile, join
import glob
import importlib

modules = glob.glob(join(dirname(__file__), "*.py"))
__all__ = [basename(f)[:-3] for f in modules if isfile(f) and not f.endswith('__init__.py')]

for module_name in __all__:
    try:
        importlib.import_module(f".{module_name}", package=__name__)
    except Exception as e:
        print(f"Error loading module {module_name}: {e}")


class DeprecatedRobotDescription:
    def raise_error(self):
        raise DeprecationWarning("Robot description moved, please use RobotDescription.current_robot_description from"
                                 " pycram.robot_description")

    @property
    def name(self):
        self.raise_error()

    @property
    def chains(self):
        self.raise_error()

    @property
    def joints(self):
        self.raise_error()

    @property
    def torso_joint(self):
        self.raise_error()

    @property
    def torso_link(self):
        self.raise_error()


robot_description = DeprecatedRobotDescription()
