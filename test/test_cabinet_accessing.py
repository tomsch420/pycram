import time
import unittest

from pycram.robot_description import RobotDescriptionManager
from pycram.world_concepts.world_object import Object
# from .datastructures.enums import WorldMode

from pycrap.ontologies import Milk, Robot, Kitchen, Cereal, Cabinet
from pycram.testing import  EmptyBulletWorldTestCase


@unittest.skip
class CabinetTestCase(EmptyBulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rdm = RobotDescriptionManager()
        rdm.load_description("pr2")
        cls.cabinet = Object("cabinet", Cabinet, "tvunit_lack_0829_boxified" + cls.extension)

    def test_cabinet(self):
        self.assertTrue(True)
