import os
import time
import unittest
from pycram.mediator_world.mediator_world import *
from pycram.mediator_world.urdf_parser import URDFParser
from pycram.ros import get_ros_package_path


class RVIZIntegrationTestCase(unittest.TestCase):

    def test_rviz(self):
        file = "resources/objects/table.urdf"
        file = os.path.join(get_ros_package_path("pycram"), file)
        parser = URDFParser(file)
        world = parser.parse()

        world_publisher = WorldPublisher(world)
        #time.sleep(100)

class ORMaticIntegrationTestCase(unittest.TestCase):
    ...



if __name__ == '__main__':
    unittest.main()
