"""ROS 2 node that adapts Mission Control packets to typed ROS topics."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import queue
import threading
from typing import cast, TypeAlias

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rover_interfaces.msg import (
    CommandHeartbeat,
    DriveCommand,
    EmergencyStopCommand,
    TankDriveCommand,
)
from std_msgs.msg import String

from mc_bridge.command_session import CommandSession, CommandStamp
from mc_bridge.packet import (
    JsonObject,
    PacketValidationError,
    decode_packet,
    validate_packet,
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
    ping_interval_sec: float
    ping_timeout_sec: float


@dataclass(frozen=True, slots=True)
class _ActiveMotion:
    session_id: int
    mode: str
    first: float
    second: float


@dataclass(frozen=True, slots=True)
class _ActiveEmergencyStop:
    session_id: int
    stop: bool


class MCBridgeNode(Node):
    """Bridge Mission Control JSON and the local ROS graph."""

    def __init__(self) -> None:
        """Create typed publishers and bounded session-aware buffers."""
        super().__init__('mc_bridge')
        self._declare_parameters()

        self.settings = BridgeSettings(
            websocket_host=self._string_parameter('websocket.host'),
            websocket_port=self._positive_integer_parameter('websocket.port'),
            websocket_path=self._string_parameter('websocket.path'),
            outbound_capacity=self._positive_integer_parameter(
                'websocket.outbound_capacity',
            ),
            ping_interval_sec=self._positive_number_parameter(
                'websocket.ping_interval_sec',
            ),
            ping_timeout_sec=self._positive_number_parameter(
                'websocket.ping_timeout_sec',
            ),
        )
        self._inbound_batch_size = self._positive_integer_parameter(
            'ros.inbound_batch_size',
        )
        self._inbound = SessionInbox(
            self._positive_integer_parameter('ros.inbound_capacity'),
        )
        self._command_session = CommandSession()
        self._disconnect_stamps: queue.SimpleQueue[CommandStamp] = (
            queue.SimpleQueue()
        )
        self._active_command_lock = threading.Lock()
        self._active_motion: _ActiveMotion | None = None
        self._active_estop: _ActiveEmergencyStop | None = None
        self._outbound_publisher: OutboundPublisher | None = None

        command_qos = QoSProfile(
            depth=self._positive_integer_parameter('ros.command_qos_depth'),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        estop_qos = QoSProfile(
            depth=self._positive_integer_parameter('ros.estop_qos_depth'),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        report_qos = QoSProfile(
            depth=self._positive_integer_parameter('ros.report_qos_depth'),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._drive_publisher = self.create_publisher(
            DriveCommand,
            self._string_parameter('ros.drive_command_topic'),
            command_qos,
        )
        self._tank_drive_publisher = self.create_publisher(
            TankDriveCommand,
            self._string_parameter('ros.tank_drive_command_topic'),
            command_qos,
        )
        self._estop_publisher = self.create_publisher(
            EmergencyStopCommand,
            self._string_parameter('ros.emergency_stop_topic'),
            estop_qos,
        )
        self._heartbeat_publisher = self.create_publisher(
            CommandHeartbeat,
            self._string_parameter('ros.heartbeat_topic'),
            command_qos,
        )
        self._report_subscription = self.create_subscription(
            String,
            self._string_parameter('ros.report_topic'),
            self._report_callback,
            report_qos,
        )
        self._inbound_timer = self.create_timer(
            self._positive_number_parameter('ros.inbound_poll_period_sec'),
            self._drain_inbound,
        )
        self._heartbeat_timer = self.create_timer(
            self._positive_number_parameter('ros.heartbeat_period_sec'),
            self._heartbeat_tick,
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
        """Start a new command identity for one WebSocket controller."""
        self._clear_active_commands()
        self._command_session.begin()
        self._inbound.begin()

    def end_ws_session(self) -> None:
        """Discard queued input and schedule an immediate neutral command."""
        self._inbound.end()
        self._clear_active_commands()
        stamp = self._command_session.end()
        if stamp is not None:
            self._disconnect_stamps.put(stamp)

    def _declare_parameters(self) -> None:
        self.declare_parameter('websocket.host', '127.0.0.1')
        self.declare_parameter('websocket.port', 3001)
        self.declare_parameter('websocket.path', '/mission-control')
        self.declare_parameter('websocket.outbound_capacity', 64)
        self.declare_parameter('websocket.ping_interval_sec', 1.0)
        self.declare_parameter('websocket.ping_timeout_sec', 1.0)
        self.declare_parameter('ros.drive_command_topic', 'comms/drive')
        self.declare_parameter(
            'ros.tank_drive_command_topic',
            'comms/tank_drive',
        )
        self.declare_parameter(
            'ros.emergency_stop_topic',
            'comms/emergency_stop',
        )
        self.declare_parameter('ros.heartbeat_topic', 'comms/heartbeat')
        self.declare_parameter('ros.report_topic', 'mc/report')
        self.declare_parameter('ros.command_qos_depth', 1)
        self.declare_parameter('ros.estop_qos_depth', 10)
        self.declare_parameter('ros.report_qos_depth', 1)
        self.declare_parameter('ros.inbound_capacity', 64)
        self.declare_parameter('ros.inbound_batch_size', 32)
        self.declare_parameter('ros.inbound_poll_period_sec', 0.01)
        self.declare_parameter('ros.heartbeat_period_sec', 0.1)

    def _drain_inbound(self) -> None:
        while True:
            try:
                stamp = self._disconnect_stamps.get_nowait()
            except queue.Empty:
                break
            self._publish_drive(stamp, 0.0, 0.0)

        for session_packet in self._inbound.take(self._inbound_batch_size):
            self._inbound.dispatch(session_packet, self._handle_ws_message)

    def _handle_ws_message(self, message: JsonObject) -> None:
        message_type = message['type']
        self._publish_heartbeat()
        stamp = self._command_session.next_stamp()
        if stamp is None:
            return
        if message_type == 'driveRequest':
            if self._estop_is_active(stamp.session_id):
                return
            straight = float(cast(int | float, message['straight']))
            steer = float(cast(int | float, message['steer']))
            self._publish_drive(
                stamp,
                straight,
                steer,
            )
            self._remember_motion(
                _ActiveMotion(stamp.session_id, 'drive', straight, steer),
            )
        elif message_type == 'tankDriveRequest':
            if self._estop_is_active(stamp.session_id):
                return
            left = float(cast(int | float, message['left']))
            right = float(cast(int | float, message['right']))
            ros_message = TankDriveCommand()
            _set_stamp(ros_message, stamp)
            ros_message.left = left
            ros_message.right = right
            self._tank_drive_publisher.publish(ros_message)
            self._remember_motion(
                _ActiveMotion(stamp.session_id, 'tank', left, right),
            )
        elif message_type == 'emergencyStopRequest':
            stop = bool(message['stop'])
            self._publish_estop(stamp, stop)
            with self._active_command_lock:
                self._active_motion = None
                self._active_estop = _ActiveEmergencyStop(
                    stamp.session_id,
                    stop,
                )
        else:
            self.get_logger().warning(
                f'Packet is valid but not implemented: {message_type}',
            )

    def _publish_drive(
        self,
        stamp: CommandStamp,
        straight: float,
        steer: float,
    ) -> None:
        message = DriveCommand()
        _set_stamp(message, stamp)
        message.straight = straight
        message.steer = steer
        self._drive_publisher.publish(message)

    def _publish_heartbeat(self) -> None:
        stamp = self._command_session.next_stamp()
        if stamp is None:
            return
        message = CommandHeartbeat()
        _set_stamp(message, stamp)
        self._heartbeat_publisher.publish(message)

    def _heartbeat_tick(self) -> None:
        self._publish_heartbeat()
        with self._active_command_lock:
            motion = self._active_motion
            estop = self._active_estop

        if motion is not None:
            stamp = self._command_session.next_stamp()
            if stamp is not None and stamp.session_id == motion.session_id:
                if motion.mode == 'drive':
                    self._publish_drive(stamp, motion.first, motion.second)
                else:
                    message = TankDriveCommand()
                    _set_stamp(message, stamp)
                    message.left = motion.first
                    message.right = motion.second
                    self._tank_drive_publisher.publish(message)
        if estop is not None:
            stamp = self._command_session.next_stamp()
            if stamp is not None and stamp.session_id == estop.session_id:
                self._publish_estop(stamp, estop.stop)

    def _publish_estop(self, stamp: CommandStamp, stop: bool) -> None:
        message = EmergencyStopCommand()
        _set_stamp(message, stamp)
        message.stop = stop
        self._estop_publisher.publish(message)

    def _remember_motion(self, motion: _ActiveMotion) -> None:
        with self._active_command_lock:
            self._active_motion = motion

    def _estop_is_active(self, session_id: int) -> bool:
        with self._active_command_lock:
            estop = self._active_estop
            return (
                estop is not None
                and estop.session_id == session_id
                and estop.stop
            )

    def _clear_active_commands(self) -> None:
        with self._active_command_lock:
            self._active_motion = None
            self._active_estop = None

    def _report_callback(self, message: String) -> None:
        publisher = self._outbound_publisher
        if publisher is None:
            return
        try:
            packet = decode_packet(message.data)
            validate_packet(packet, direction='report')
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
            or not math.isfinite(value)
            or value <= 0.0
        ):
            raise ValueError(f'Parameter {name} must be a positive number')
        return float(value)


def _set_stamp(
    message: CommandHeartbeat
    | DriveCommand
    | EmergencyStopCommand
    | TankDriveCommand,
    stamp: CommandStamp,
) -> None:
    message.session_id = stamp.session_id
    message.sequence = stamp.sequence


def create_node() -> MCBridgeNode:
    """Create the bridge node after rclpy has been initialized."""
    if not rclpy.ok():
        raise RuntimeError('rclpy must be initialized before creating bridge')
    return MCBridgeNode()
