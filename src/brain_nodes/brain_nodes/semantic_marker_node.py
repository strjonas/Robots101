"""Semantic markers: draws the named places from the semantic map in RViz.

Publishes:   /brain/semantic_markers (visualization_msgs/MarkerArray, 1 Hz)

Pure visualization. Each target becomes a dot with its name above it, and the
patrol route is drawn as a line through the waypoints in order.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from brain_nodes.constants import DEFAULT_SEMANTIC_MAP_PATH, TOPIC_SEMANTIC_MARKERS
from brain_nodes.semantic_map import SemanticMap


class SemanticMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_marker_node")
        self.declare_parameter("semantic_map_path", str(DEFAULT_SEMANTIC_MAP_PATH))
        self.semantic_map_path = Path(str(self.get_parameter("semantic_map_path").value))
        self.publisher = self.create_publisher(MarkerArray, TOPIC_SEMANTIC_MARKERS, 1)
        self.create_timer(1.0, self._publish)

    def _publish(self) -> None:
        # Reload every time so targets added with record-waypoint show up straight away.
        semantic_map = SemanticMap.load(self.semantic_map_path)
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        def new_marker(marker_id: int, marker_type: int) -> Marker:
            marker = Marker()
            marker.header.frame_id = semantic_map.frame_id
            marker.header.stamp = stamp
            marker.ns = "semantic_map"
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.color.a = 1.0
            return marker

        for index, target in enumerate(semantic_map.targets):
            dot = new_marker(2 * index, Marker.SPHERE)
            dot.pose.position.x = target.x
            dot.pose.position.y = target.y
            dot.pose.position.z = 0.05
            dot.scale.x = dot.scale.y = dot.scale.z = 0.18
            dot.color.r, dot.color.g, dot.color.b = 0.1, 0.6, 1.0
            markers.markers.append(dot)

            label = new_marker(2 * index + 1, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = target.x
            label.pose.position.y = target.y
            label.pose.position.z = 0.45
            label.scale.z = 0.22
            label.color.r = label.color.g = label.color.b = 1.0
            label.text = target.name
            markers.markers.append(label)

        route = list(semantic_map.patrol_targets())
        if len(route) > 1:
            line = new_marker(10_000, Marker.LINE_STRIP)
            line.scale.x = 0.03
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.8, 0.0, 0.6
            # Close the loop: after the last waypoint patrol returns to the first.
            line.points = [Point(x=target.x, y=target.y, z=0.03) for target in route + route[:1]]
            markers.markers.append(line)

        self.publisher.publish(markers)


def main() -> None:
    rclpy.init()
    node = SemanticMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
