import os
import time
import unittest

from pycram.datastructures.enums import WorldMode
from pycram.mediator_world.geometry import Box
from pycram.mediator_world.pose import PoseStamped
from pycram.mediator_world.urdf_parser import URDFParser
from pycram.mediator_world.worlds.bullet_world import BulletWorld
from pycram.ros import get_ros_package_path


class BulletWorldTestCase(unittest.TestCase):

    def test_creation(self):


        file = "resources/objects/table.urdf"
        file = os.path.join(get_ros_package_path("pycram"), file)
        parsed = URDFParser(file).parse()

        world = BulletWorld(render_mode=WorldMode.GUI)
        world.add_from_world(parsed)

        world.create_multi_body(world.get_link_by_name("left_front_leg"))

        time.sleep(1)




if __name__ == '__main__':
    unittest.main()
