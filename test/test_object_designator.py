import time
import unittest
from bullet_world_testcase import BulletWorldTestCase
from pycram.description import ObjectDescription
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.ontology.ontology import OntologyManager, SOMA_ONTOLOGY_IRI
from pycram.process_module import ProcessModule
from pycram.robot_description import RobotDescriptionManager
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
import pycram.tasktree


class TestObjectDesignator(BulletWorldTestCase):

    def test_object_grounding(self):
        description = ObjectDesignatorDescription(["milk"], [ObjectType.MILK])
        obj = description.ground()

        self.assertEqual(obj.name, "milk")
        self.assertEqual(obj.obj_type, ObjectType.MILK)

    def test_frozen_copy(self):
        description = ObjectDesignatorDescription(["milk"], [ObjectType.MILK])
        obj = description.ground()

        frozen_copy = obj.frozen_copy()
        self.assertEqual(obj.pose, frozen_copy.pose)


class SemanticObjectDesignatorTestCase(unittest.TestCase):
    world: BulletWorld
    viz_marker_publisher: VizMarkerPublisher
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        rdm = RobotDescriptionManager()
        rdm.load_description("pr2")
        cls.world = BulletWorld(mode=WorldMode.DIRECT)
        ProcessModule.execution_delay = False
        cls.viz_marker_publisher = VizMarkerPublisher()
        OntologyManager(SOMA_ONTOLOGY_IRI)

    def setUp(self):
        self.world.reset_world()

    def tearDown(self):
        pycram.tasktree.reset_tree()
        time.sleep(0.05)
        self.world.reset_world()

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()




if __name__ == '__main__':
    unittest.main()
