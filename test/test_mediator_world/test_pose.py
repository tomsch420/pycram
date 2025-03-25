import inspect
import unittest

import ormatic.ormatic
from sqlalchemy import create_engine, select
from sqlalchemy.orm import registry, Session

import pycram.mediator_world.pose
import copy


class PoseTestCase(unittest.TestCase):

    def test_copy(self):
        pose = pycram.mediator_world.pose.Pose()
        pose.position.x = 1
        pose.position.y = 2
        pose.position.z = 3
        pose.orientation.x = 4
        pose.orientation.y = 5
        pose.orientation.z = 6
        pose.orientation.w = 7

        pose_copy = copy.copy(pose)

        self.assertEqual(pose.position.x, pose_copy.position.x)
        self.assertEqual(pose.position.y, pose_copy.position.y)
        self.assertEqual(pose.position.z, pose_copy.position.z)
        self.assertEqual(pose.orientation.x, pose_copy.orientation.x)
        self.assertEqual(pose.orientation.y, pose_copy.orientation.y)
        self.assertEqual(pose.orientation.z, pose_copy.orientation.z)
        self.assertEqual(pose.orientation.w, pose_copy.orientation.w)

class ORMaticIntegrationTestCase(unittest.TestCase):

    def test_integration(self):

        # list all classes of module
        classes = inspect.getmembers(pycram.mediator_world.pose, inspect.isclass)
        classes = [c[1] for c in classes]

        # create ormatic tool
        mapper_registry = registry()
        ormatic_tool = ormatic.ormatic.ORMatic(classes, mapper_registry)
        ormatic_tool.make_all_tables()

        # create a session
        engine = create_engine("sqlite:///:memory:")
        ormatic_tool.make_all_tables()
        session = Session(engine)
        mapper_registry.metadata.create_all(engine)

        # create an object
        pose = pycram.mediator_world.pose.Pose()
        session.add(pose)
        session.commit()

        # query the object
        query = session.scalars(select(pycram.mediator_world.pose.Pose)).all()



if __name__ == '__main__':
    unittest.main()
