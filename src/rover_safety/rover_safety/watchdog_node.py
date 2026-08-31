"""ROS 2 adapter for the rover-side command safety gate."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rover_interfaces.msg import (
    CommandHeartbeat,
    DriveCommand,
    EmergencyStopCommand,
    SafetyState,
    TankDriveCommand,
    WheelCommand,
)

from rover_safety.safety_gate import SafetyGate, SafetySnapshot


class RoverWatchdogNode(Node):
    """Publish only motion commands approved by the local safety gate."""

    def __init__(self) -> None:
        """Configure command inputs, safe output, and watchdog timer."""
        super().__init__('rover_command_watchdog')
        self.declare_parameter('heartbeat_timeout_sec', 0.5)
        self.declare_parameter('command_timeout_sec', 0.3)
        self.declare_parameter('check_period_sec', 0.05)
        self.declare_parameter('topics.heartbeat', 'comms/heartbeat')
        self.declare_parameter('topics.drive', 'comms/drive')
        self.declare_parameter('topics.tank_drive', 'comms/tank_drive')
        self.declare_parameter('topics.emergency_stop', 'comms/emergency_stop')
        self.declare_parameter('topics.safe_wheels', 'drive/safe_wheels')
        self.declare_parameter('topics.safety_state', 'safety/state')

        self._gate = SafetyGate(
            self._positive_number('heartbeat_timeout_sec'),
            self._positive_number('command_timeout_sec'),
        )
        commands = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        estop = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._wheel_publisher = self.create_publisher(
            WheelCommand,
            self._topic('topics.safe_wheels'),
            commands,
        )
        self._state_publisher = self.create_publisher(
            SafetyState,
            self._topic('topics.safety_state'),
            state,
        )
        self._subscriptions = [
            self.create_subscription(
                CommandHeartbeat,
                self._topic('topics.heartbeat'),
                self._heartbeat_callback,
                commands,
            ),
            self.create_subscription(
                DriveCommand,
                self._topic('topics.drive'),
                self._drive_callback,
                commands,
            ),
            self.create_subscription(
                TankDriveCommand,
                self._topic('topics.tank_drive'),
                self._tank_drive_callback,
                commands,
            ),
            self.create_subscription(
                EmergencyStopCommand,
                self._topic('topics.emergency_stop'),
                self._emergency_stop_callback,
                estop,
            ),
        ]
        self._timer = self.create_timer(
            self._positive_number('check_period_sec'),
            self._check_timeouts,
        )
        self._publish(self._gate.snapshot)
        self.get_logger().info('Rover command watchdog initialized')

    def _heartbeat_callback(self, message: CommandHeartbeat) -> None:
        before = self._gate.snapshot
        self._gate.receive_heartbeat(
            message.session_id,
            message.sequence,
            time.monotonic(),
        )
        self._publish_if_changed(before)

    def _drive_callback(self, message: DriveCommand) -> None:
        before = self._gate.snapshot
        self._gate.receive_drive(
            message.session_id,
            message.sequence,
            message.straight,
            message.steer,
            time.monotonic(),
        )
        self._publish_if_changed(before)

    def _tank_drive_callback(self, message: TankDriveCommand) -> None:
        before = self._gate.snapshot
        self._gate.receive_tank_drive(
            message.session_id,
            message.sequence,
            message.left,
            message.right,
            time.monotonic(),
        )
        self._publish_if_changed(before)

    def _emergency_stop_callback(self, message: EmergencyStopCommand) -> None:
        before = self._gate.snapshot
        self._gate.receive_emergency_stop(
            message.session_id,
            message.sequence,
            message.stop,
        )
        self._publish_if_changed(before)

    def _check_timeouts(self) -> None:
        before = self._gate.snapshot
        self._gate.check_timeouts(time.monotonic())
        self._publish_if_changed(before)

    def _publish_if_changed(self, before: SafetySnapshot) -> None:
        after = self._gate.snapshot
        if after != before:
            self._publish(after)

    def _publish(self, snapshot: SafetySnapshot) -> None:
        wheels = WheelCommand()
        wheels.left = snapshot.wheels.left
        wheels.right = snapshot.wheels.right
        self._wheel_publisher.publish(wheels)

        state = SafetyState()
        state.active_session_id = snapshot.active_session_id
        state.emergency_stop_latched = snapshot.emergency_stop_latched
        state.heartbeat_timed_out = snapshot.heartbeat_timed_out
        state.command_timed_out = snapshot.command_timed_out
        self._state_publisher.publish(state)

    def _topic(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'Parameter {name} must be a non-empty string')
        return value

    def _positive_number(self, name: str) -> float:
        value = self.get_parameter(name).value
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0.0
        ):
            raise ValueError(f'Parameter {name} must be a positive number')
        return float(value)


def main(args: list[str] | None = None) -> None:
    """Run the rover watchdog node."""
    rclpy.init(args=args)
    node: RoverWatchdogNode | None = None
    try:
        node = RoverWatchdogNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
