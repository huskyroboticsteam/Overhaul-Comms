"""ROS 2 adapter between Mission Control packets and typed rover topics."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import threading
from typing import cast, TypeAlias

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rover_interfaces.msg import (
    ArmIKCommand,
    ArmIKEnabledReport,
    CameraCommand,
    CameraFrameReport,
    CameraStreamReport,
    CommandHeartbeat,
    CommandSessionStart,
    DriveCommand,
    EmergencyStopCommand,
    JointPositionCommand,
    JointPositionReport,
    JointPowerCommand,
    MountedPeripheralReport,
    OperationModeCommand,
    RoverPositionReport,
    ServoPositionCommand,
    ServoPositionReport,
    StepperTurnAngleCommand,
    TankDriveCommand,
    WaypointNavCommand,
)

from mc_bridge.camera_worker import (
    CameraFrame,
    CameraReport,
    CameraReportWorker,
    CameraStreamFrame,
)
from mc_bridge.command_session import CommandSession, CommandStamp
from mc_bridge.packet import JsonObject, PacketValidationError, validate_packet
from mc_bridge.session_inbox import SessionInbox

OutboundPublisher: TypeAlias = Callable[[JsonObject], bool]
RequestHandler: TypeAlias = Callable[[JsonObject, CommandStamp], None]

# Mission Control indexes reports directly into these current UI keys. Legacy
# science names remain valid inputs, but emitting them can crash that client.
_MISSION_CONTROL_JOINTS = frozenset(
    {
        'armBase',
        'shoulder',
        'elbow',
        'forearm',
        'wristPitch',
        'wristRoll',
        'hand',
        'handActuator',
        'ikUp',
        'ikForward',
    },
)
_MISSION_CONTROL_SERVOS = frozenset({'mast'})
_MISSION_CONTROL_CAMERAS = frozenset({'mast', 'hand', 'wrist'})
_IK_POSITION_JOINTS = frozenset({'ikUp', 'ikForward'})
_MAX_CAMERA_FRAME_BYTES = 12 * 1024 * 1024
_MAX_CAMERA_STREAM_BYTES = 4 * 1024 * 1024
_MAX_CAMERA_FPS = 60


@dataclass(frozen=True, slots=True)
class BridgeSettings:
    """Runtime settings needed by the WebSocket server."""

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


@dataclass(frozen=True, slots=True)
class _ActiveJointPower:
    session_id: int
    power: float


class MCBridgeNode(Node):
    """Translate validated packets without depending on network transport."""

    def __init__(self) -> None:
        """Create typed publishers, subscriptions, and bounded workers."""
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
        self._active_command_lock = threading.Lock()
        self._active_motion: _ActiveMotion | None = None
        self._active_estop: _ActiveEmergencyStop | None = None
        self._active_joint_power: dict[str, _ActiveJointPower] = {}

        self._report_lock = threading.Lock()
        self._connected_session_id = 0
        self._retained_reports: dict[str, JsonObject] = {}
        self._camera_frames: dict[str, CommandStamp] = {}
        self._camera_streams: dict[str, CommandStamp] = {}
        self._outbound_publisher: OutboundPublisher | None = None
        self._camera_max_frame_bytes = self._positive_integer_parameter(
            'camera.max_frame_bytes',
        )
        self._camera_max_stream_bytes = self._positive_integer_parameter(
            'camera.max_stream_bytes',
        )
        if self._camera_max_frame_bytes > _MAX_CAMERA_FRAME_BYTES:
            raise ValueError('camera.max_frame_bytes exceeds the ROS bound')
        if self._camera_max_stream_bytes > _MAX_CAMERA_STREAM_BYTES:
            raise ValueError('camera.max_stream_bytes exceeds the ROS bound')
        self._camera_max_fps = self._positive_integer_parameter(
            'camera.max_fps',
        )
        if self._camera_max_fps > _MAX_CAMERA_FPS:
            raise ValueError('camera.max_fps exceeds the packet contract')
        self._camera_worker = CameraReportWorker(
            self._camera_report_ready,
            capacity=self._positive_integer_parameter(
                'camera.worker_capacity',
            ),
            error_handler=self._camera_worker_failed,
        )
        self._worker_closed = False

        commands = QoSProfile(
            depth=self._positive_integer_parameter('ros.command_qos_depth'),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        estop = QoSProfile(
            depth=self._positive_integer_parameter('ros.estop_qos_depth'),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        telemetry = QoSProfile(
            depth=self._positive_integer_parameter('ros.telemetry_qos_depth'),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_state = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        video = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._drive_publisher = self.create_publisher(
            DriveCommand,
            self._topic('drive_command'),
            commands,
        )
        self._tank_drive_publisher = self.create_publisher(
            TankDriveCommand,
            self._topic('tank_drive_command'),
            commands,
        )
        self._estop_publisher = self.create_publisher(
            EmergencyStopCommand,
            self._topic('emergency_stop'),
            estop,
        )
        self._heartbeat_publisher = self.create_publisher(
            CommandHeartbeat,
            self._topic('heartbeat'),
            commands,
        )
        self._session_start_publisher = self.create_publisher(
            CommandSessionStart,
            self._topic('session_start'),
            commands,
        )
        self._operation_mode_publisher = self.create_publisher(
            OperationModeCommand,
            self._topic('operation_mode'),
            commands,
        )
        self._joint_power_publisher = self.create_publisher(
            JointPowerCommand,
            self._topic('joint_power'),
            commands,
        )
        self._joint_position_publisher = self.create_publisher(
            JointPositionCommand,
            self._topic('joint_position'),
            commands,
        )
        self._arm_ik_publisher = self.create_publisher(
            ArmIKCommand,
            self._topic('arm_ik'),
            commands,
        )
        self._servo_position_publisher = self.create_publisher(
            ServoPositionCommand,
            self._topic('servo_position'),
            commands,
        )
        self._stepper_publisher = self.create_publisher(
            StepperTurnAngleCommand,
            self._topic('stepper_turn_angle'),
            commands,
        )
        self._waypoint_publisher = self.create_publisher(
            WaypointNavCommand,
            self._topic('waypoint_nav'),
            commands,
        )
        self._camera_command_publisher = self.create_publisher(
            CameraCommand,
            self._topic('camera_command'),
            commands,
        )

        self.create_subscription(
            RoverPositionReport,
            self._topic('rover_position_report'),
            self._rover_position_callback,
            telemetry,
        )
        self.create_subscription(
            JointPositionReport,
            self._topic('joint_position_report'),
            self._joint_position_callback,
            telemetry,
        )
        self.create_subscription(
            ServoPositionReport,
            self._topic('servo_position_report'),
            self._servo_position_callback,
            telemetry,
        )
        self.create_subscription(
            MountedPeripheralReport,
            self._topic('mounted_peripheral_report'),
            self._mounted_peripheral_callback,
            static_state,
        )
        self.create_subscription(
            ArmIKEnabledReport,
            self._topic('arm_ik_enabled_report'),
            self._arm_ik_enabled_callback,
            static_state,
        )
        self.create_subscription(
            CameraFrameReport,
            self._topic('camera_frame_report'),
            self._camera_frame_callback,
            video,
        )
        self.create_subscription(
            CameraStreamReport,
            self._topic('camera_stream_report'),
            self._camera_stream_callback,
            video,
        )

        self._request_handlers: dict[str, RequestHandler] = {
            'driveRequest': self._handle_drive,
            'tankDriveRequest': self._handle_tank_drive,
            'emergencyStopRequest': self._handle_emergency_stop,
            'operationModeRequest': self._handle_operation_mode,
            'jointPowerRequest': self._handle_joint_power,
            'jointPositionRequest': self._handle_joint_position,
            'armIKRequest': self._handle_arm_ik,
            'servoPositionRequest': self._handle_servo_position,
            'stepperTurnAngleRequest': self._handle_stepper,
            'waypointNavRequest': self._handle_waypoint,
            'cameraFrameRequest': self._handle_camera,
            'cameraStreamOpenRequest': self._handle_camera,
            'cameraStreamCloseRequest': self._handle_camera,
        }
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
        """Start a command identity and replay only retained static state."""
        self._clear_active_commands()
        session_id = self._command_session.begin()
        self._inbound.begin()
        session_start = CommandSessionStart()
        session_start.session_id = session_id
        self._session_start_publisher.publish(session_start)
        with self._report_lock:
            self._connected_session_id = session_id
            publisher = self._outbound_publisher
            if publisher is not None:
                for packet in self._retained_reports.values():
                    publisher(dict(packet))

    def end_ws_session(self) -> None:
        """Discard queued input and schedule immediate neutral controls."""
        latch_estop = self._inbound.end()
        joints = self._clear_active_commands()
        self._camera_worker.clear()
        with self._report_lock:
            self._connected_session_id = 0
            self._camera_frames.clear()
            active_streams = tuple(self._camera_streams)
            self._camera_streams.clear()
        for camera in active_streams:
            close_stamp = self._command_session.next_stamp()
            if close_stamp is None:
                break
            message = CameraCommand()
            _set_stamp(message, close_stamp)
            message.action = CameraCommand.STREAM_CLOSE
            message.camera = camera
            message.fps = 0
            self._camera_command_publisher.publish(message)
        stamp = self._command_session.end()
        if stamp is not None:
            if latch_estop:
                self._publish_estop(stamp, True)
            self._publish_neutral(stamp, joints)

    def destroy_node(self) -> bool:
        """Stop the camera worker before destroying ROS entities."""
        if not self._worker_closed:
            self._worker_closed = True
            self._camera_worker.close()
        return bool(super().destroy_node())

    def _declare_parameters(self) -> None:
        parameters: tuple[tuple[str, object], ...] = (
            ('websocket.host', '127.0.0.1'),
            ('websocket.port', 3001),
            ('websocket.path', '/mission-control'),
            ('websocket.outbound_capacity', 64),
            ('websocket.ping_interval_sec', 1.0),
            ('websocket.ping_timeout_sec', 1.0),
            ('ros.command_qos_depth', 1),
            ('ros.estop_qos_depth', 10),
            ('ros.telemetry_qos_depth', 1),
            ('ros.inbound_capacity', 64),
            ('ros.inbound_batch_size', 32),
            ('ros.inbound_poll_period_sec', 0.01),
            ('ros.heartbeat_period_sec', 0.1),
            ('camera.worker_capacity', 6),
            ('camera.max_fps', _MAX_CAMERA_FPS),
            ('camera.max_frame_bytes', _MAX_CAMERA_FRAME_BYTES),
            ('camera.max_stream_bytes', _MAX_CAMERA_STREAM_BYTES),
            ('topics.drive_command', 'comms/drive'),
            ('topics.tank_drive_command', 'comms/tank_drive'),
            ('topics.emergency_stop', 'comms/emergency_stop'),
            ('topics.heartbeat', 'comms/heartbeat'),
            ('topics.session_start', 'comms/session_start'),
            ('topics.operation_mode', 'comms/operation_mode'),
            ('topics.joint_power', 'comms/joint_power'),
            ('topics.joint_position', 'comms/joint_position'),
            ('topics.arm_ik', 'comms/arm_ik'),
            ('topics.servo_position', 'comms/servo_position'),
            ('topics.stepper_turn_angle', 'comms/stepper_turn_angle'),
            ('topics.waypoint_nav', 'comms/waypoint_nav'),
            ('topics.camera_command', 'comms/camera'),
            ('topics.rover_position_report', 'telemetry/rover_position'),
            ('topics.joint_position_report', 'telemetry/joint_position'),
            ('topics.servo_position_report', 'telemetry/servo_position'),
            ('topics.mounted_peripheral_report', 'telemetry/peripheral'),
            ('topics.arm_ik_enabled_report', 'telemetry/arm_ik'),
            ('topics.camera_frame_report', 'camera/frame'),
            ('topics.camera_stream_report', 'camera/stream'),
        )
        for name, default in parameters:
            self.declare_parameter(name, default)

    def _drain_inbound(self) -> None:
        for session_packet in self._inbound.take(self._inbound_batch_size):
            self._inbound.dispatch(session_packet, self._handle_ws_message)

    def _handle_ws_message(self, message: JsonObject) -> None:
        self._publish_heartbeat()
        stamp = self._command_session.next_stamp()
        if stamp is None:
            return
        message_type = cast(str, message['type'])
        handler = self._request_handlers.get(message_type)
        if handler is None:
            self.get_logger().warning(
                f'Packet is valid but not implemented: {message_type}',
            )
            return
        handler(message, stamp)

    def _handle_drive(self, packet: JsonObject, stamp: CommandStamp) -> None:
        if self._estop_is_active(stamp.session_id):
            return
        straight = _number(packet, 'straight')
        steer = _number(packet, 'steer')
        self._publish_drive(stamp, straight, steer)
        with self._active_command_lock:
            self._active_motion = _ActiveMotion(
                stamp.session_id,
                'drive',
                straight,
                steer,
            )

    def _handle_tank_drive(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        if self._estop_is_active(stamp.session_id):
            return
        left = _number(packet, 'left')
        right = _number(packet, 'right')
        self._publish_tank_drive(stamp, left, right)
        with self._active_command_lock:
            self._active_motion = _ActiveMotion(
                stamp.session_id,
                'tank',
                left,
                right,
            )

    def _handle_emergency_stop(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        joints = self._clear_motion_commands()
        stop = bool(packet['stop'])
        self._publish_estop(stamp, stop)
        neutral_stamp = self._command_session.next_stamp() or stamp
        self._publish_neutral(neutral_stamp, joints)
        with self._active_command_lock:
            self._active_estop = (
                _ActiveEmergencyStop(stamp.session_id, True)
                if stop
                else None
            )

    def _handle_operation_mode(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        joints = self._clear_motion_commands()
        message = OperationModeCommand()
        _set_stamp(message, stamp)
        mode = cast(str, packet['mode'])
        message.mode = (
            OperationModeCommand.AUTONOMOUS
            if mode == 'autonomous'
            else OperationModeCommand.TELEOPERATION
        )
        self._operation_mode_publisher.publish(message)
        neutral_stamp = self._command_session.next_stamp() or stamp
        self._publish_neutral(neutral_stamp, joints)

    def _handle_joint_power(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        if self._estop_is_active(stamp.session_id):
            return
        joint = cast(str, packet['joint'])
        power = _number(packet, 'power')
        self._publish_joint_power(stamp, joint, power)
        with self._active_command_lock:
            if power == 0.0:
                self._active_joint_power.pop(joint, None)
            else:
                self._active_joint_power[joint] = _ActiveJointPower(
                    stamp.session_id,
                    power,
                )

    def _handle_joint_position(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        if self._estop_is_active(stamp.session_id):
            return
        message = JointPositionCommand()
        _set_stamp(message, stamp)
        joint = cast(str, packet['joint'])
        message.joint = joint
        message.position = _number(packet, 'position')
        message.unit = (
            JointPositionCommand.METERS
            if joint in _IK_POSITION_JOINTS
            else JointPositionCommand.DEGREES
        )
        self._joint_position_publisher.publish(message)

    def _handle_arm_ik(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        enabled = bool(packet['enabled'])
        if enabled and self._estop_is_active(stamp.session_id):
            return
        message = ArmIKCommand()
        _set_stamp(message, stamp)
        message.enabled = enabled
        self._arm_ik_publisher.publish(message)

    def _handle_servo_position(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        if self._estop_is_active(stamp.session_id):
            return
        message = ServoPositionCommand()
        _set_stamp(message, stamp)
        message.servo = cast(str, packet['servo'])
        message.position_deg = _number(packet, 'position')
        self._servo_position_publisher.publish(message)

    def _handle_stepper(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        if self._estop_is_active(stamp.session_id):
            return
        message = StepperTurnAngleCommand()
        _set_stamp(message, stamp)
        message.stepper = cast(str, packet['stepper'])
        message.angle_deg = int(cast(int, packet['angle']))
        self._stepper_publisher.publish(message)

    def _handle_waypoint(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        if self._estop_is_active(stamp.session_id):
            return
        message = WaypointNavCommand()
        _set_stamp(message, stamp)
        message.latitude_deg = _number(packet, 'latitude')
        message.longitude_deg = _number(packet, 'longitude')
        message.is_approximate = bool(packet['isApproximate'])
        message.is_gate = bool(packet['isGate'])
        self._waypoint_publisher.publish(message)

    def _handle_camera(
        self,
        packet: JsonObject,
        stamp: CommandStamp,
    ) -> None:
        packet_type = cast(str, packet['type'])
        camera = cast(str, packet['camera'])
        message = CameraCommand()
        _set_stamp(message, stamp)
        message.camera = camera
        if packet_type == 'cameraFrameRequest':
            message.action = CameraCommand.CAPTURE
            message.fps = 0
            with self._report_lock:
                self._camera_frames[camera] = stamp
        elif packet_type == 'cameraStreamOpenRequest':
            fps = int(cast(int, packet['fps']))
            if fps > self._camera_max_fps:
                self.get_logger().warning('Ignoring camera FPS above limit')
                return
            message.action = CameraCommand.STREAM_OPEN
            message.fps = fps
            with self._report_lock:
                self._camera_streams[camera] = stamp
        else:
            message.action = CameraCommand.STREAM_CLOSE
            message.fps = 0
            with self._report_lock:
                self._camera_streams.pop(camera, None)
        self._camera_command_publisher.publish(message)

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

    def _publish_tank_drive(
        self,
        stamp: CommandStamp,
        left: float,
        right: float,
    ) -> None:
        message = TankDriveCommand()
        _set_stamp(message, stamp)
        message.left = left
        message.right = right
        self._tank_drive_publisher.publish(message)

    def _publish_estop(self, stamp: CommandStamp, stop: bool) -> None:
        message = EmergencyStopCommand()
        _set_stamp(message, stamp)
        message.stop = stop
        self._estop_publisher.publish(message)

    def _publish_joint_power(
        self,
        stamp: CommandStamp,
        joint: str,
        power: float,
    ) -> None:
        message = JointPowerCommand()
        _set_stamp(message, stamp)
        message.joint = joint
        message.power = power
        self._joint_power_publisher.publish(message)

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
            joint_power = dict(self._active_joint_power)

        if motion is not None:
            stamp = self._command_session.next_stamp()
            if stamp is not None and stamp.session_id == motion.session_id:
                if motion.mode == 'drive':
                    self._publish_drive(stamp, motion.first, motion.second)
                else:
                    self._publish_tank_drive(
                        stamp,
                        motion.first,
                        motion.second,
                    )
        for joint, active in joint_power.items():
            stamp = self._command_session.next_stamp()
            if stamp is not None and stamp.session_id == active.session_id:
                self._publish_joint_power(stamp, joint, active.power)
        if estop is not None and estop.stop:
            stamp = self._command_session.next_stamp()
            if stamp is not None and stamp.session_id == estop.session_id:
                self._publish_estop(stamp, True)

    def _publish_neutral(
        self,
        stamp: CommandStamp,
        joints: tuple[str, ...],
    ) -> None:
        self._publish_drive(stamp, 0.0, 0.0)
        for joint in joints:
            self._publish_joint_power(stamp, joint, 0.0)

    def _clear_motion_commands(self) -> tuple[str, ...]:
        with self._active_command_lock:
            joints = tuple(self._active_joint_power)
            self._active_motion = None
            self._active_joint_power.clear()
            return joints

    def _clear_active_commands(self) -> tuple[str, ...]:
        with self._active_command_lock:
            joints = tuple(self._active_joint_power)
            self._active_motion = None
            self._active_estop = None
            self._active_joint_power.clear()
            return joints

    def _estop_is_active(self, session_id: int) -> bool:
        with self._active_command_lock:
            estop = self._active_estop
            return bool(
                estop is not None
                and estop.session_id == session_id
                and estop.stop
            )

    def _rover_position_callback(self, message: RoverPositionReport) -> None:
        self._send_report(
            {
                'type': 'roverPositionReport',
                'orientW': message.orientation_w,
                'orientX': message.orientation_x,
                'orientY': message.orientation_y,
                'orientZ': message.orientation_z,
                'lon': message.longitude_deg,
                'lat': message.latitude_deg,
                'alt': message.altitude_m,
                'recency': message.recency_sec,
            },
        )

    def _joint_position_callback(self, message: JointPositionReport) -> None:
        if message.joint not in _MISSION_CONTROL_JOINTS:
            return
        expected_unit = (
            JointPositionReport.METERS
            if message.joint in _IK_POSITION_JOINTS
            else JointPositionReport.DEGREES
        )
        if message.unit != expected_unit:
            self.get_logger().warning('Ignoring joint position with bad unit')
            return
        self._send_report(
            {
                'type': 'jointPositionReport',
                'joint': message.joint,
                'position': message.position,
            },
        )

    def _servo_position_callback(self, message: ServoPositionReport) -> None:
        if message.servo not in _MISSION_CONTROL_SERVOS:
            return
        self._send_report(
            {
                'type': 'servoPositionReport',
                'servo': message.servo,
                'position': message.position_deg,
            },
        )

    def _mounted_peripheral_callback(
        self,
        message: MountedPeripheralReport,
    ) -> None:
        peripherals: dict[int, str | None] = {
            MountedPeripheralReport.NONE: None,
            MountedPeripheralReport.ARM: 'arm',
            MountedPeripheralReport.SCIENCE: 'science',
        }
        peripheral = peripherals.get(message.peripheral)
        if message.peripheral not in peripherals:
            self.get_logger().warning('Ignoring unknown mounted peripheral')
            return
        self._send_report(
            {
                'type': 'mountedPeripheralReport',
                'peripheral': peripheral,
            },
            retain=True,
        )

    def _arm_ik_enabled_callback(self, message: ArmIKEnabledReport) -> None:
        self._send_report(
            {
                'type': 'armIKEnabledReport',
                'enabled': message.enabled,
            },
            retain=True,
        )

    def _camera_frame_callback(self, message: CameraFrameReport) -> None:
        if not self._camera_report_is_expected(
            message.session_id,
            message.request_sequence,
            message.camera,
            single_frame=True,
        ):
            return
        pose = (
            message.latitude_deg,
            message.longitude_deg,
            message.altitude_m,
            message.orientation_x,
            message.orientation_y,
            message.orientation_z,
            message.orientation_w,
        )
        if not all(math.isfinite(value) for value in pose):
            self.get_logger().warning('Dropping invalid camera frame pose')
            return
        data = bytes(message.jpeg_data)
        if len(data) > self._camera_max_frame_bytes:
            self.get_logger().warning('Dropping oversized camera frame')
            return
        self._camera_worker.offer(
            CameraFrame(
                session_id=message.session_id,
                request_sequence=message.request_sequence,
                camera=message.camera,
                jpeg_data=data,
                latitude_deg=message.latitude_deg,
                longitude_deg=message.longitude_deg,
                altitude_m=message.altitude_m,
                orientation_x=message.orientation_x,
                orientation_y=message.orientation_y,
                orientation_z=message.orientation_z,
                orientation_w=message.orientation_w,
            ),
        )

    def _camera_stream_callback(self, message: CameraStreamReport) -> None:
        if not self._camera_report_is_expected(
            message.session_id,
            message.request_sequence,
            message.camera,
            single_frame=False,
        ):
            return
        data = bytes(message.data)
        unit_lengths = tuple(int(length) for length in message.unit_lengths)
        if len(data) > self._camera_max_stream_bytes:
            self.get_logger().warning('Dropping oversized camera stream frame')
            return
        if message.available and (
            any(length < 1 for length in unit_lengths)
            or sum(unit_lengths) != len(data)
        ):
            self.get_logger().warning('Dropping malformed camera stream frame')
            return
        self._camera_worker.offer(
            CameraStreamFrame(
                session_id=message.session_id,
                request_sequence=message.request_sequence,
                camera=message.camera,
                available=message.available,
                data=data,
                unit_lengths=unit_lengths,
            ),
        )

    def _camera_report_is_expected(
        self,
        session_id: int,
        request_sequence: int,
        camera: str,
        *,
        single_frame: bool,
    ) -> bool:
        if camera not in _MISSION_CONTROL_CAMERAS:
            return False
        with self._report_lock:
            requests = (
                self._camera_frames
                if single_frame
                else self._camera_streams
            )
            expected = requests.get(camera)
            return bool(
                session_id == self._connected_session_id
                and expected is not None
                and expected.sequence == request_sequence
            )

    def _camera_report_ready(
        self,
        report: CameraReport,
        packet: JsonObject,
    ) -> None:
        with self._report_lock:
            if report.session_id != self._connected_session_id:
                return
            if isinstance(report, CameraFrame):
                expected = self._camera_frames.get(report.camera)
                if (
                    expected is None
                    or expected.sequence != report.request_sequence
                ):
                    return
                del self._camera_frames[report.camera]
            else:
                expected = self._camera_streams.get(report.camera)
                if (
                    expected is None
                    or expected.sequence != report.request_sequence
                ):
                    return
            self._send_report(packet, validate=False)

    def _send_report(
        self,
        packet: JsonObject,
        *,
        retain: bool = False,
        validate: bool = True,
    ) -> None:
        if validate:
            try:
                validate_packet(packet, direction='report')
            except PacketValidationError as error:
                self.get_logger().warning(f'Ignoring invalid report: {error}')
                return
        if retain:
            packet_type = cast(str, packet['type'])
            with self._report_lock:
                self._retained_reports[packet_type] = dict(packet)
        publisher = self._outbound_publisher
        if publisher is not None:
            publisher(packet)

    def _camera_worker_failed(self, error: Exception) -> None:
        self.get_logger().error(f'Camera report conversion failed: {error}')

    def _topic(self, suffix: str) -> str:
        return self._string_parameter(f'topics.{suffix}')

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


StampedMessage: TypeAlias = (
    ArmIKCommand
    | CameraCommand
    | CommandHeartbeat
    | DriveCommand
    | EmergencyStopCommand
    | JointPositionCommand
    | JointPowerCommand
    | OperationModeCommand
    | ServoPositionCommand
    | StepperTurnAngleCommand
    | TankDriveCommand
    | WaypointNavCommand
)


def _set_stamp(message: StampedMessage, stamp: CommandStamp) -> None:
    message.session_id = stamp.session_id
    message.sequence = stamp.sequence


def _number(packet: JsonObject, field: str) -> float:
    return float(cast(int | float, packet[field]))


def create_node() -> MCBridgeNode:
    """Create the bridge node after rclpy has been initialized."""
    if not rclpy.ok():
        raise RuntimeError('rclpy must be initialized before creating bridge')
    return MCBridgeNode()
