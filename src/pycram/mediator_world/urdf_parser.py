from .mediator_world import World
import urdfpy

class URDFParser:
    """
    Class to parse URDF files to world objects.
    """

    file_path: str

    def __init__(self, file: str):
        self.file_path = file

    def parse(self) -> World:
        world = World()

        urdf = urdfpy.URDF.load(self.file_path)

        for link in urdf.links:
            print(link)




        return world

