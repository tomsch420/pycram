import os
import unittest

from pycram.mediator_world.robot import PR2
from pycram.mediator_world.urdf_parser import *
from pycram.ros import get_ros_package_path


class PR2TestCase(unittest.TestCase):

    def test_pr2_from_urdf(self):
        pr2 = PR2.make_from_urdf()
