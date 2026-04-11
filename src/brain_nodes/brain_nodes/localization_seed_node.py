from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from brain_nodes.constants import DEFAULT_SEMANTIC_MAP_PATH
from brain_nodes.semantic_map import SemanticMap, SemanticTarget


class LocalizationSeedNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_seed_node")
        self.declare_parameter("semantic_map_path", str(DEFAULT_SEMANTIC_MAP_PATH))
        self.declare_parameter("initial_target_name", "foyer_spawn")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("max_attempts", 20)
        self.declare_parameter("covariance_xy", 0.25)
        self.declare_parameter("covariance_yaw", 0.0685)

        semantic_map_path = Path(str(self.get_parameter("semantic_map_path").value))
        initial_target_name = str(self.get_parameter("initial_target_name").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.max_attempts = int(self.get_parameter("max_attempts").value)
        self.covariance_xy = float(self.get_parameter("covariance_xy").value)
        self.covariance_yaw = float(self.get_parameter("covariance_yaw").value)

        semantic_map = SemanticMap.load(semantic_map_path)
        self.target = self._select_target(semantic_map, initial_target_name)
        self.localized = False
        self.publish_attempts = 0
        self.max_attempts_logged = False

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10)

        if self.target is not None:
            self.get_logger().info(
                f"Localization seed target -> {self.target.name} "
                f"({self.target.x:.2f}, {self.target.y:.2f}, yaw={self.target.yaw:.2f})"
            )
            self.create_timer(self.publish_period_sec, self._tick)
        else:
            self.get_logger().error("No semantic target available to seed AMCL.")

    def _select_target(self, semantic_map: SemanticMap, requested_name: str) -> SemanticTarget | None:
        target = semantic_map.get(requested_name)
        if target is not None:
            return target

        patrol_targets = list(semantic_map.patrol_targets())
        if patrol_targets:
            fallback = patrol_targets[0]
            self.get_logger().warning(
                f"Semantic target '{requested_name}' not found. Falling back to patrol target '{fallback.name}'."
            )
            return fallback

        described_targets = list(semantic_map.targets.values())
        if described_targets:
            fallback = described_targets[0]
            self.get_logger().warning(
                f"Semantic target '{requested_name}' not found. Falling back to first semantic target '{fallback.name}'."
            )
            return fallback
        return None

    def _amcl_cb(self, _: PoseWithCovarianceStamped) -> None:
        if not self.localized:
            self.get_logger().info("AMCL pose received. Localization seeded successfully.")
        self.localized = True

    def _tick(self) -> None:
        if self.localized or self.target is None:
            return
        if self.publish_attempts >= self.max_attempts:
            if not self.max_attempts_logged:
                self.get_logger().warning("Localization seed attempts exhausted while waiting for AMCL pose.")
                self.max_attempts_logged = True
            return

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = self.target.x
        message.pose.pose.position.y = self.target.y
        message.pose.pose.orientation.z = math.sin(self.target.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self.target.yaw / 2.0)
        message.pose.covariance[0] = self.covariance_xy
        message.pose.covariance[7] = self.covariance_xy
        message.pose.covariance[35] = self.covariance_yaw

        self.publish_attempts += 1
        self.publisher.publish(message)
        self.get_logger().info(
            f"Published AMCL initial pose attempt {self.publish_attempts}/{self.max_attempts} "
            f"for target '{self.target.name}'."
        )


def main() -> None:
    rclpy.init()
    node = LocalizationSeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
