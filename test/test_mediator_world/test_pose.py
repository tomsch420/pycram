import copy
import unittest

import ormatic.ormatic
from sqlalchemy import create_engine, select
from sqlalchemy.orm import registry, Session

import pycram.mediator_world.pose


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

        self.assertEqual(pose, pose_copy)


class ORMaticIntegrationTestCase(unittest.TestCase):

    def test_integration(self):
        # list all classes of module
        classes = [pycram.mediator_world.pose.Vector3, pycram.mediator_world.pose.Quaternion,
                   pycram.mediator_world.pose.Pose, pycram.mediator_world.pose.Header,
                   pycram.mediator_world.pose.PoseStamped]

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
