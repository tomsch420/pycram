from dataclasses import dataclass

from typing_extensions import List

@dataclass
class Link:
    ...


@dataclass
class Joint:
    ...

@dataclass
class World:
    links: List[Link]
    joints: List[Joint]