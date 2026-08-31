"""ROS 2 node that adapts Mission Control packets to ROS topics."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from typing import TypeAlias

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from mc_bridge.packet import (
    JsonObject,
    PacketValidationError,
    decode_packet,
    encode_packet,
    validate_normalized_fields,
)
from mc_bridge.session_inbox import SessionInbox

OutboundPublisher: TypeAlias = Callable[[JsonObject], bool]


@dataclass(frozen=True, slots=True)
class BridgeSettings:
    """Runtime settings read from ROS parameters."""

    websocket_host: str
    websocket_port: int
    websocket_path: str
    outbound_capacity: int


class MCBridgeNode(Node):
    """Bridge Mission Control JSON and the local ROS graph."""

    def __init__(self) -> None:
        """Create publishers, subscriptions, and bounded packet buffers."""
        super().__init__('mc_bridge')

        self.declare_parameter('websocket.host', '127.0.0.1')
        self.declare_parameter('websocket.port', 3001)
        self.declare_parameter('websocket.path', '/mission-control')
        self.declare_parameter('websocket.outbound_capacity', 64)
        self.declare_parameter('ros.drive_command_topic', 'mc/drive/cmd')
        self.declare_parameter('ros.report_topic', 'mc/report')
        self.declare_parameter('ros.command_qos_depth', 1)
        self.declare_parameter('ros.report_qos_depth', 1)
        self.declare_parameter('ros.inbound_capacity', 64)
        self.declare_parameter('ros.inbound_batch_size', 32)
        self.declare_parameter('ros.inbound_poll_period_sec', 0.01)

        self.settings = BridgeSettings(
            websocket_host=self._string_parameter('websocket.host'),
            websocket_port=self._positive_integer_parameter('websocket.port'),
            websocket_path=self._string_parameter('websocket.path'),
            outbound_capacity=self._positive_integer_parameter(
                'websocket.outbound_capacity',
            ),
        )
        inbound_capacity = self._positive_integer_parameter(
            'ros.inbound_capacity',
        )
        self._inbound_batch_size = self._positive_integer_parameter(
            'ros.inbound_batch_size',
        )
        self._inbound = SessionInbox(inbound_capacity)
        self._outbound_publisher: OutboundPublisher | None = None

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self._positive_integer_parameter('ros.command_qos_depth'),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        report_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self._positive_integer_parameter('ros.report_qos_depth'),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        drive_command_topic = self._string_parameter('ros.drive_command_topic')
        report_topic = self._string_parameter('ros.report_topic')
        self._drive_publisher = self.create_publisher(
            String,
            drive_command_topic,
            command_qos,
        )
        self._report_subscription = self.create_subscription(
            String,
            report_topic,
            self._report_callback,
            report_qos,
        )
        self._inbound_timer = self.create_timer(
            self._positive_number_parameter('ros.inbound_poll_period_sec'),
            self._drain_inbound,
        )
        self.get_logger().info('Mission Control bridge initialized')

    def set_outbound_publisher(
        self,
        publisher: OutboundPublisher | None,
    ) -> None:
        """Attach or detach the thread-safe WebSocket publisher."""
        self._outbound_publisher = publisher

    def enqueue_ws_message(self, message: JsonObject) -> bool:
        """Queue validated WebSocket input for execution in the ROS thread."""
        accepted = self._inbound.offer(message)
        if not accepted:
            self.get_logger().error('ROS input queue is full or inactive')
        return accepted

    def begin_ws_session(self) -> None:
        """Start a new input generation for one WebSocket controller."""
        self._inbound.begin()

    def end_ws_session(self) -> None:
        """Invalidate and discard every packet from the closed controller."""
        self._inbound.end()

    def _drain_inbound(self) -> None:
        pending = self._inbound.take(self._inbound_batch_size)
        for session_packet in pending:
            self._inbound.dispatch(session_packet, self._handle_ws_message)

    def _handle_ws_message(self, message: JsonObject) -> None:
        message_type = message['type']
        if message_type == 'driveRequest':
            try:
                validate_normalized_fields(message, 'straight', 'steer')
            except PacketValidationError as error:
                self.get_logger().warning(
                    f'Ignoring invalid driveRequest: {error}',
                )
                return
            ros_message = String()
            ros_message.data = encode_packet(message)
            self._drive_publisher.publish(ros_message)
        elif message_type == 'cameraStreamOpenRequest':
            self.get_logger().info('Received camera stream open request')
        elif message_type == 'cameraStreamCloseRequest':
            self.get_logger().info('Received camera stream close request')
        else:
            self.get_logger().warning(
                f'Unhandled Mission Control message type: {message_type}',
            )

    def _report_callback(self, message: String) -> None:
        publisher = self._outbound_publisher
        if publisher is None:
            return
        try:
            packet = decode_packet(message.data)
        except PacketValidationError as error:
            self.get_logger().warning(
                f'Ignoring invalid report packet: {error}',
            )
            return
        publisher(packet)

    def _string_parameter(self, name: str) -> str:
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            raise ValueError(f'Parameter {name} must be a non-empty string')
        return value

    def _positive_integer_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise ValueError(f'Parameter {name} must be a positive integer')
        return value

    def _positive_number_parameter(self, name: str) -> float:
        value = self.get_parameter(name).value
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or value <= 0.0
        ):
            raise ValueError(f'Parameter {name} must be a positive number')
        try:
            result = float(value)
        except OverflowError as error:
            raise ValueError(
                f'Parameter {name} must be a finite number',
            ) from error
        if not math.isfinite(result):
            raise ValueError(f'Parameter {name} must be a finite number')
        return result


def create_node() -> MCBridgeNode:
    """Create the bridge node after rclpy has been initialized."""
    if not rclpy.ok():
        raise RuntimeError(
            'rclpy must be initialized before creating the bridge',
        )
    return MCBridgeNode()
