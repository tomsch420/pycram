import unittest
from pycram.mediator_world.mediator_world import *


class WorldTestCase(unittest.TestCase):


    def test_creation(self):
        world = World()
        l1 = Link("l1")
        l2 = Link("l2")
        j1 = Joint(JointType.REVOLUTE, l1, l2)
        self.assertEqual(len(world.links), 0)
        world.add_joint(j1)
        self.assertEqual(len(world.links), 2)
        self.assertEqual(len(world.joints), 1)




if __name__ == '__main__':
    unittest.main()
