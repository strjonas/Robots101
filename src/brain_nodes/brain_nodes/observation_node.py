from __future__ import annotations

from pathlib import Path

import rclpy
from brain_interfaces.msg import ExecutorStatus, ObservationSummary, TrackedObjectArray
from brain_interfaces.srv import RecordSemanticTarget
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from brain_nodes.constants import (
    DEFAULT_SEMANTIC_MAP_PATH,
    SERVICE_RECORD_SEMANTIC_TARGET,
    TOPIC_EXECUTOR_STATUS,
    TOPIC_OBSERVATION_SUMMARY,
    TOPIC_TRACKED_OBJECTS,
)
from brain_nodes.math_utils import finite_min, quaternion_to_yaw
from brain_nodes.semantic_map import SemanticMap


class ObservationNode(Node):
    def __init__(self) -> None:
        super().__init__("observation_node")
        self.declare_parameter("semantic_map_path", str(DEFAULT_SEMANTIC_MAP_PATH))
        self.declare_parameter("semantic_match_radius_m", 1.5)
        self.semantic_map_path = Path(str(self.get_parameter("semantic_map_path").value))
        self.semantic_match_radius = float(self.get_parameter("semantic_match_radius_m").value)
        self.semantic_map = SemanticMap.load(self.semantic_map_path)

        self.publisher = self.create_publisher(ObservationSummary, TOPIC_OBSERVATION_SUMMARY, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(TrackedObjectArray, TOPIC_TRACKED_OBJECTS, self._tracked_objects_cb, 10)
        self.create_subscription(ExecutorStatus, TOPIC_EXECUTOR_STATUS, self._executor_status_cb, 10)
        self.create_service(RecordSemanticTarget, SERVICE_RECORD_SEMANTIC_TARGET, self._record_target)
        self.create_timer(0.5, self._publish_summary)

        self.odom = None
        self.amcl_pose = None
        self.scan = None
        self.tracked_objects = TrackedObjectArray()
        self.executor_status = ExecutorStatus()

    def _odom_cb(self, message: Odometry) -> None:
        self.odom = message

    def _amcl_cb(self, message: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = message

    def _scan_cb(self, message: LaserScan) -> None:
        self.scan = message

    def _tracked_objects_cb(self, message: TrackedObjectArray) -> None:
        self.tracked_objects = message

    def _executor_status_cb(self, message: ExecutorStatus) -> None:
        self.executor_status = message

    def _active_pose(self) -> tuple[float, float, float, bool]:
        if self.amcl_pose is not None:
            pose = self.amcl_pose.pose.pose
            yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            return float(pose.position.x), float(pose.position.y), yaw, True
        if self.odom is not None:
            pose = self.odom.pose.pose
            yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            return float(pose.position.x), float(pose.position.y), yaw, False
        return 0.0, 0.0, 0.0, False

    def _scan_sectors(self) -> tuple[float, float, float, float, bool, bool]:
        if self.scan is None or not self.scan.ranges:
            return float("inf"), float("inf"), float("inf"), float("inf"), False, False

        ranges = self.scan.ranges
        quarter = max(1, len(ranges) // 4)
        front = finite_min(ranges[:quarter] + ranges[-quarter:])
        left = finite_min(ranges[quarter: quarter * 2])
        rear = finite_min(ranges[quarter * 2: quarter * 3])
        right = finite_min(ranges[quarter * 3:])
        obstacle_close = front < 0.4
        collision_imminent = front < 0.22
        return front, left, right, rear, obstacle_close, collision_imminent

    def _publish_summary(self) -> None:
        x, y, yaw, localized = self._active_pose()
        front, left, right, rear, obstacle_close, collision_imminent = self._scan_sectors()

        message = ObservationSummary()
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose_x = x
        message.pose_y = y
        message.pose_yaw = yaw
        message.semantic_location = self.semantic_map.nearest_name(x, y, max_distance=self.semantic_match_radius)
        message.nav_state = self.executor_status.detail
        message.localization_ok = localized
        message.obstacle_close = obstacle_close
        message.collision_imminent = collision_imminent
        message.front_min_range = front
        message.left_min_range = left
        message.right_min_range = right
        message.rear_min_range = rear
        message.active_executor = self.executor_status.executor_name
        message.last_executor_status = self.executor_status.active_action_type
        message.tracked_objects = list(self.tracked_objects.objects)
        message.visible_labels = list(dict.fromkeys(obj.label for obj in self.tracked_objects.objects))
        self.publisher.publish(message)

    def _record_target(
        self,
        request: RecordSemanticTarget.Request,
        response: RecordSemanticTarget.Response,
    ) -> RecordSemanticTarget.Response:
        x, y, yaw, _ = self._active_pose()
        if not request.name.strip():
            response.saved = False
            response.message = "Name cannot be empty."
            return response

        self.semantic_map.upsert_target(request.name.strip(), x, y, yaw, request.tag.strip())
        self.semantic_map.save(self.semantic_map_path)
        response.saved = True
        response.message = f"Saved semantic target '{request.name.strip()}' to {self.semantic_map_path}"
        self.get_logger().info(response.message)
        return response


def main() -> None:
    rclpy.init()
    node = ObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
