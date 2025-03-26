import os
import time
import unittest
from pycram.mediator_world.mediator_world import *
from pycram.mediator_world.pose import Header, Vector3, Quaternion
from pycram.mediator_world.urdf_parser import URDFParser
from pycram.ros import get_ros_package_path


class TransformTestCase(unittest.TestCase):

    def test_transform(self):
        w = World()
        p1 = Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
        p1 = PoseStamped(pose=p1, header=Header())
        l1 = Link("l1", origin=p1)

        p2 = Pose(Vector3(3, 6, 9), Quaternion(0, 0, -1, 1))
        p2 = PoseStamped(pose=p2, header=Header())
        l2 = Link("l2", origin=p2)

        w.add_link(l1)
        w.add_link(l2)

        result = w._transformer.transform_pose(l2.origin, l2, l1)

        correct_position = Vector3(2, 4, 6)
        correct_orientation = Quaternion(0, 0, -1, 1)
        self.assertEqual(result.header.frame_id, "l1")
        self.assertEqual(result.position, correct_position)
        self.assertEqual(result.orientation, correct_orientation)



class RVIZIntegrationTestCase(unittest.TestCase):

    def test_rviz(self):
        file = "resources/robots/pr2.urdf"
        file = os.path.join(get_ros_package_path("pycram"), file)
        parser = URDFParser(file)
        world = parser.parse()
        world_publisher = WorldPublisher(world)
        time.sleep(100)

class ORMaticIntegrationTestCase(unittest.TestCase):
    ...



if __name__ == '__main__':
    unittest.main()
