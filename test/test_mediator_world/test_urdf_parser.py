import os
import unittest
from pycram.mediator_world.urdf_parser import *
from pycram.ros import get_ros_package_path


class URDFTestCase(unittest.TestCase):

    def test_table_parsing(self):
        file = "resources/objects/table.urdf"
        file = os.path.join(get_ros_package_path("pycram"), file)
        parser = URDFParser(file)

        world = parser.parse()
        self.assertEqual(len(world.joints), 5)
        self.assertTrue(all([joint.type == JointType.FIXED for joint in world.joints]))
        self.assertEqual(len(world.links), 7)

if __name__ == '__main__':
    unittest.main()
