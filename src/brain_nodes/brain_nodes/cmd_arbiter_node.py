"""Command arbiter: the last gate before a velocity command reaches the robot.

Subscribes:  /brain/cmd_vel_manual    (Twist, keyboard teleop)
             /brain/cmd_vel_executor  (Twist, skill executor)
             /brain/cmd_vel_nav       (TwistStamped, Nav2)
             /brain/control_mode      (who is allowed to drive)
             /brain/safety_stop       (obstacle right ahead)
Publishes:   /brain/cmd_vel_final     (Twist, the one command that wins)
             /brain/active_cmd_source (String, for debugging)

The decision rules live in arbiter_logic.py. If the winning source goes quiet,
the arbiter sends zero velocity, so a crashed node cannot leave the robot driving.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from brain_interfaces.msg import ControlMode
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from brain_nodes.arbiter_logic import (
    SOURCE_EXECUTOR,
    SOURCE_MANUAL,
    SOURCE_NAV,
    limit_forward_speed,
    select_source,
)
from brain_nodes.constants import (
    TOPIC_ACTIVE_CMD_SOURCE,
    TOPIC_CMD_VEL,
    TOPIC_CMD_VEL_EXECUTOR,
    TOPIC_CMD_VEL_MANUAL,
    TOPIC_CMD_VEL_NAV,
    TOPIC_CONTROL_MODE,
    TOPIC_SAFETY_STOP,
)


@dataclass
class TimedTwist:
    message: Twist = field(default_factory=Twist)
    stamp_ns: int = 0


class CmdArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_arbiter")
        self.declare_parameter("manual_timeout_sec", 0.5)
        self.declare_parameter("nav_timeout_sec", 0.5)
        self.declare_parameter("executor_timeout_sec", 0.3)

        self.manual_timeout_sec = float(self.get_parameter("manual_timeout_sec").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.executor_timeout_sec = float(self.get_parameter("executor_timeout_sec").value)

        self.cmd_publisher = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)
        self.source_publisher = self.create_publisher(String, TOPIC_ACTIVE_CMD_SOURCE, 10)

        self.create_subscription(Twist, TOPIC_CMD_VEL_MANUAL, self._manual_cb, 10)
        self.create_subscription(TwistStamped, TOPIC_CMD_VEL_NAV, self._nav_cb, 10)
        self.create_subscription(Twist, TOPIC_CMD_VEL_EXECUTOR, self._executor_cb, 10)
        self.create_subscription(ControlMode, TOPIC_CONTROL_MODE, self._mode_cb, 10)
        self.create_subscription(Bool, TOPIC_SAFETY_STOP, self._safety_cb, 10)

        self.manual_cmd = TimedTwist()
        self.nav_cmd = TimedTwist()
        self.executor_cmd = TimedTwist()
        self.current_mode = ControlMode()
        self.current_mode.mode = ControlMode.IDLE
        self.safety_stop = False
        self.last_source = ""
        self.last_forward_blocked = False
        self.ticks_since_source_publish = 0

        self.create_timer(0.05, self._publish_selected_cmd)

    def _manual_cb(self, message: Twist) -> None:
        self.manual_cmd = TimedTwist(message=message, stamp_ns=self.get_clock().now().nanoseconds)

    def _nav_cb(self, message: TwistStamped) -> None:
        self.nav_cmd = TimedTwist(message=message.twist, stamp_ns=self.get_clock().now().nanoseconds)

    def _executor_cb(self, message: Twist) -> None:
        self.executor_cmd = TimedTwist(message=message, stamp_ns=self.get_clock().now().nanoseconds)

    def _mode_cb(self, message: ControlMode) -> None:
        self.current_mode = message

    def _safety_cb(self, message: Bool) -> None:
        self.safety_stop = message.data

    def _is_fresh(self, timed_twist: TimedTwist, timeout_sec: float) -> bool:
        age_sec = (self.get_clock().now().nanoseconds - timed_twist.stamp_ns) / 1e9
        return timed_twist.stamp_ns > 0 and age_sec <= timeout_sec

    def _publish_selected_cmd(self) -> None:
        source = select_source(
            mode=self.current_mode.mode,
            manual_fresh=self._is_fresh(self.manual_cmd, self.manual_timeout_sec),
            executor_fresh=self._is_fresh(self.executor_cmd, self.executor_timeout_sec),
            nav_fresh=self._is_fresh(self.nav_cmd, self.nav_timeout_sec),
        )
        commands = {
            SOURCE_MANUAL: self.manual_cmd.message,
            SOURCE_EXECUTOR: self.executor_cmd.message,
            SOURCE_NAV: self.nav_cmd.message,
        }
        selected = commands.get(source, Twist())

        output = Twist()
        output.linear.x = limit_forward_speed(selected.linear.x, self.safety_stop)
        output.angular.z = selected.angular.z
        self.cmd_publisher.publish(output)

        forward_blocked = output.linear.x != selected.linear.x
        if forward_blocked != self.last_forward_blocked:
            self.last_forward_blocked = forward_blocked
            if forward_blocked:
                self.get_logger().warning(f"Obstacle ahead: blocking forward motion from '{source}'")

        # Announce changes immediately, and repeat once a second for tools that join late.
        self.ticks_since_source_publish += 1
        if source != self.last_source:
            self.get_logger().info(f"Arbiter source -> {source}")
        if source != self.last_source or self.ticks_since_source_publish >= 20:
            self.last_source = source
            self.ticks_since_source_publish = 0
            self.source_publisher.publish(String(data=source))


def main() -> None:
    rclpy.init()
    node = CmdArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
