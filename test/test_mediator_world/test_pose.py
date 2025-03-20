import inspect
import unittest

import ormatic.ormatic
from sqlalchemy import create_engine, select
from sqlalchemy.orm import registry, Session

import pycram.mediator_world.pose


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
