"""ROS2 Python Package for Hydra-ROS."""

from typing import Optional

import rclpy
import spark_dsg as dsg
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Header

from hydra_msgs.msg import DsgUpdate


class DsgPublisher:
    """Class for publishing a scene graph from python."""

    def __init__(self, node: Node, topic: str, publish_mesh: bool = True):
        """Construct a sender."""
        self._publish_mesh = publish_mesh

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = node.create_publisher(DsgUpdate, topic, qos_profile)

    def publish(
        self, G, stamp: Optional[rclpy.time.Time] = None, frame_id: str = "odom"
    ):
        """Send a graph."""
        now = stamp if stamp is not None else rclpy.clock.Clock().now()
        header = Header()
        header.stamp = Time(
            sec=now.nanoseconds // 10**9, nanosec=now.nanoseconds % 10**9
        )
        header.frame_id = frame_id
        self.publish_with_header(G, header)

    def publish_with_header(self, G, header: Header):
        """Send a graph."""
        msg = DsgUpdate()
        msg.header = header
        msg.layer_contents = G.to_binary(self._publish_mesh)
        msg.full_update = True
        self._pub.publish(msg)


class DsgSubscriber:
    """Class for receiving a scene graph in python."""

    def __init__(self, node: Node, topic: str, callback, qos=None):
        """Construct a DSG Receiver."""
        self._callback = callback
        self._graph_set = False
        self._graph = None
        self._logger = node.get_logger()

        qos_profile = qos or QoSProfile(depth=1)
        self._sub = node.create_subscription(
            DsgUpdate, topic, self._handle_update, qos_profile
        )

    def _handle_update(self, msg: DsgUpdate):
        if not msg.full_update:
            raise NotImplementedError("Partial updates not implemented yet")

        size_bytes = len(msg.layer_contents)
        self._logger.debug(f"Received dsg update message of {size_bytes} bytes")

        if not self._graph_set:
            self._graph = dsg.DynamicSceneGraph.from_binary(
                msg.layer_contents.tobytes()
            )
            self._graph_set = True
        else:
            self._graph.update_from_binary(msg.layer_contents.tobytes())

        self._callback(msg.header, self._graph)
