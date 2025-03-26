from __future__ import annotations

import atexit
import threading
import time

from visualization_msgs.msg import MarkerArray

from pycram.mediator_world.mediator_world import World
from pycram.ros import create_publisher


class WorldPublisher:
    """
    Publishes the visuals of every link in the world to a ros topic.
    """

    world: World
    """
    The world to read the links from.
    """

    topic_name: str
    """
    The name of the topic to publish the visuals to.
    """

    interval: float
    """
    The interval in seconds at which to publish the visuals.
    """

    def __init__(self, world: World, topic_name="/pycram/viz_marker", interval=0.1, reference_frame="map"):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval
        self.reference_frame = reference_frame

        self.pub = create_publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish, name="WorldPublisher")

        self.kill_event = threading.Event()
        self.world = world
        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            marker_array = self._make_marker_array()
            self.pub.publish(marker_array)
            time.sleep(self.interval)

    def _make_marker_array(self) -> MarkerArray:
        """
        Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
        creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
        tag of the URDF.

        :return: An Array of Visualization Marker
        """
        marker_array = MarkerArray()
        for link in self.world.links:
            for shape in link.visual:
                marker = shape.ros_message()
                marker_array.markers.append(marker)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()
