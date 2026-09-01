"""ROS-level tests for typed Mission Control packet translation."""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
import threading

import pytest

rclpy = pytest.importorskip('rclpy')

from rover_interfaces.msg import (  # noqa: E402
    ArmIKCommand,
    ArmIKEnabledReport,
    CameraCommand,
    CameraFrameReport,
    CameraStreamReport,
    DriveCommand,
    CommandSessionStart,
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

from mc_bridge.bridge_node import MCBridgeNode  # noqa: E402
from mc_bridge.packet import JsonObject  # noqa: E402


class _PublisherRecorder:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


@pytest.fixture
def bridge() -> Iterator[MCBridgeNode]:
    """Create and reliably destroy one initialized ROS bridge."""
    rclpy.init()
    node = MCBridgeNode()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _record_commands(
    bridge: MCBridgeNode,
) -> dict[str, _PublisherRecorder]:
    publishers = {
        name: _PublisherRecorder()
        for name in (
            'drive',
            'tank_drive',
            'estop',
            'heartbeat',
            'session_start',
            'operation_mode',
            'joint_power',
            'joint_position',
            'arm_ik',
            'servo_position',
            'stepper',
            'waypoint',
            'camera_command',
        )
    }
    for name, publisher in publishers.items():
        setattr(bridge, f'_{name}_publisher', publisher)
    return publishers


def _dispatch(bridge: MCBridgeNode, packet: JsonObject) -> None:
    assert bridge.enqueue_ws_message(packet)
    bridge._drain_inbound()


def _implemented_fixtures() -> dict[str, list[JsonObject]]:
    path = (
        Path(__file__).resolve().parents[3]
        / 'protocol'
        / 'fixtures'
        / 'implemented_packets.json'
    )
    return json.loads(path.read_text(encoding='utf-8'))


def test_all_requests_publish_typed_commands(bridge: MCBridgeNode) -> None:
    """Every v1 request maps to its explicit typed ROS interface."""
    publishers = _record_commands(bridge)
    bridge.begin_ws_session()
    assert isinstance(
        publishers['session_start'].messages[-1],
        CommandSessionStart,
    )

    requests: list[JsonObject] = [
        {'type': 'operationModeRequest', 'mode': 'autonomous'},
        {
            'type': 'jointPositionRequest',
            'joint': 'elbow',
            'position': 45.0,
        },
        {
            'type': 'jointPositionRequest',
            'joint': 'ikUp',
            'position': 0.1,
        },
        {
            'type': 'jointPositionRequest',
            'joint': 'ikForward',
            'position': 0.2,
        },
        {'type': 'armIKRequest', 'enabled': True},
        {
            'type': 'servoPositionRequest',
            'servo': 'mast',
            'position': 90.0,
        },
        {
            'type': 'stepperTurnAngleRequest',
            'stepper': 'mast',
            'angle': -45,
        },
        {
            'type': 'waypointNavRequest',
            'latitude': 47.655,
            'longitude': -122.307,
            'isApproximate': False,
            'isGate': True,
        },
        {'type': 'cameraFrameRequest', 'camera': 'mast'},
        {
            'type': 'cameraStreamOpenRequest',
            'camera': 'mast',
            'fps': 20,
        },
        {'type': 'cameraStreamCloseRequest', 'camera': 'mast'},
        {'type': 'driveRequest', 'straight': 0.5, 'steer': -0.25},
        {'type': 'tankDriveRequest', 'left': -0.25, 'right': 0.25},
        {'type': 'jointPowerRequest', 'joint': 'elbow', 'power': 0.5},
        {'type': 'emergencyStopRequest', 'stop': True},
    ]
    for request in requests:
        _dispatch(bridge, request)

    fixtures = _implemented_fixtures()
    request_types = {request['type'] for request in requests}
    assert request_types == {
        packet['type'] for packet in fixtures['requests']
    }
    assert request_types == set(bridge._request_handlers)

    mode = publishers['operation_mode'].messages[-1]
    assert isinstance(mode, OperationModeCommand)
    assert mode.mode == OperationModeCommand.AUTONOMOUS

    joint_position = publishers['joint_position'].messages[0]
    assert isinstance(joint_position, JointPositionCommand)
    assert (joint_position.joint, joint_position.position) == (
        'elbow',
        45.0,
    )
    assert joint_position.unit == JointPositionCommand.DEGREES
    ik_position = publishers['joint_position'].messages[1]
    assert isinstance(ik_position, JointPositionCommand)
    assert (ik_position.joint, ik_position.position) == ('ikUp', 0.1)
    assert ik_position.unit == JointPositionCommand.METERS
    ik_forward = publishers['joint_position'].messages[2]
    assert isinstance(ik_forward, JointPositionCommand)
    assert (ik_forward.joint, ik_forward.position) == ('ikForward', 0.2)
    assert ik_forward.unit == JointPositionCommand.METERS

    ik = publishers['arm_ik'].messages[-1]
    assert isinstance(ik, ArmIKCommand)
    assert ik.enabled

    servo = publishers['servo_position'].messages[-1]
    assert isinstance(servo, ServoPositionCommand)
    assert (servo.servo, servo.position_deg) == ('mast', 90.0)

    stepper = publishers['stepper'].messages[-1]
    assert isinstance(stepper, StepperTurnAngleCommand)
    assert (stepper.stepper, stepper.angle_deg) == ('mast', -45)

    waypoint = publishers['waypoint'].messages[-1]
    assert isinstance(waypoint, WaypointNavCommand)
    assert (waypoint.latitude_deg, waypoint.longitude_deg) == (
        47.655,
        -122.307,
    )
    assert waypoint.is_gate and not waypoint.is_approximate

    camera_messages = publishers['camera_command'].messages
    assert all(
        isinstance(message, CameraCommand)
        for message in camera_messages
    )
    assert [message.action for message in camera_messages] == [
        CameraCommand.CAPTURE,
        CameraCommand.STREAM_OPEN,
        CameraCommand.STREAM_CLOSE,
    ]
    assert camera_messages[1].fps == 20

    drive = publishers['drive'].messages[-2]
    assert isinstance(drive, DriveCommand)
    assert (drive.straight, drive.steer) == (0.5, -0.25)

    tank = publishers['tank_drive'].messages[-1]
    assert isinstance(tank, TankDriveCommand)
    assert (tank.left, tank.right) == (-0.25, 0.25)

    joint_power = publishers['joint_power'].messages[-2]
    assert isinstance(joint_power, JointPowerCommand)
    assert (joint_power.joint, joint_power.power) == ('elbow', 0.5)

    estop = publishers['estop'].messages[-1]
    assert isinstance(estop, EmergencyStopCommand)
    assert estop.stop
    assert len(publishers['heartbeat'].messages) == len(requests)


def test_all_reports_preserve_mission_control_shapes(
    bridge: MCBridgeNode,
) -> None:
    """Typed rover reports become bounded legacy WebSocket packets."""
    fixtures = _implemented_fixtures()
    publishers = _record_commands(bridge)
    packets: list[JsonObject] = []
    packet_ready = threading.Event()

    def record(packet: JsonObject) -> bool:
        packets.append(dict(packet))
        packet_ready.set()
        return True

    bridge.set_outbound_publisher(record)
    bridge.begin_ws_session()

    pose = RoverPositionReport()
    pose.orientation_w = 1.0
    pose.longitude_deg = -122.307
    pose.latitude_deg = 47.655
    pose.altitude_m = 30.0
    pose.recency_sec = 0.05
    bridge._rover_position_callback(pose)

    joint = JointPositionReport()
    joint.joint = 'elbow'
    joint.position = 45.0
    joint.unit = JointPositionReport.DEGREES
    bridge._joint_position_callback(joint)
    joint.joint = 'ikUp'
    joint.position = 0.1
    joint.unit = JointPositionReport.METERS
    bridge._joint_position_callback(joint)
    joint.joint = 'ikForward'
    joint.position = 0.2
    bridge._joint_position_callback(joint)
    joint.joint = 'unknown'
    bridge._joint_position_callback(joint)

    servo = ServoPositionReport()
    servo.servo = 'mast'
    servo.position_deg = 90.0
    bridge._servo_position_callback(servo)
    servo.servo = 'microscope'
    bridge._servo_position_callback(servo)

    peripheral = MountedPeripheralReport()
    peripheral.peripheral = MountedPeripheralReport.ARM
    bridge._mounted_peripheral_callback(peripheral)

    ik = ArmIKEnabledReport()
    ik.enabled = True
    bridge._arm_ik_enabled_callback(ik)

    _dispatch(bridge, {'type': 'cameraFrameRequest', 'camera': 'mast'})
    capture = publishers['camera_command'].messages[-1]
    assert isinstance(capture, CameraCommand)
    frame = CameraFrameReport()
    frame.session_id = capture.session_id
    frame.request_sequence = capture.sequence
    frame.camera = 'mast'
    frame.jpeg_data = [106, 112, 101, 103]
    frame.latitude_deg = 47.655
    frame.longitude_deg = -122.307
    frame.altitude_m = 30.0
    frame.orientation_w = 1.0
    packet_ready.clear()
    bridge._camera_frame_callback(frame)
    assert packet_ready.wait(timeout=1.0)

    _dispatch(
        bridge,
        {
            'type': 'cameraStreamOpenRequest',
            'camera': 'mast',
            'fps': 20,
        },
    )
    opening = publishers['camera_command'].messages[-1]
    assert isinstance(opening, CameraCommand)
    stream = CameraStreamReport()
    stream.session_id = opening.session_id
    stream.request_sequence = opening.sequence
    stream.camera = 'mast'
    stream.available = True
    stream.data = [0, 0, 0, 1, 101, 136]
    stream.unit_lengths = [6]
    packet_ready.clear()
    bridge._camera_stream_callback(stream)
    assert packet_ready.wait(timeout=1.0)

    assert [packet['type'] for packet in packets] == [
        'roverPositionReport',
        'jointPositionReport',
        'jointPositionReport',
        'jointPositionReport',
        'servoPositionReport',
        'mountedPeripheralReport',
        'armIKEnabledReport',
        'cameraFrameReport',
        'cameraStreamReport',
    ]
    assert packets[0]['lon'] == -122.307
    assert packets[1]['position'] == 45.0
    assert packets[2]['position'] == 0.1
    assert packets[3]['position'] == 0.2
    assert packets[4]['position'] == 90.0
    assert packets[5]['peripheral'] == 'arm'
    assert packets[6]['enabled'] is True
    assert packets[7]['data'] == 'anBlZw=='
    assert packets[8]['data'] == [[0, 0, 0, 1, 101, 136]]
    assert {packet['type'] for packet in packets} == {
        packet['type']
        for packet in fixtures['reports']
    }


def test_disconnect_cannot_drop_a_pending_estop(
    bridge: MCBridgeNode,
) -> None:
    """An accepted stop is published even if ROS has not polled it yet."""
    publishers = _record_commands(bridge)
    bridge.begin_ws_session()
    assert bridge.enqueue_ws_message(
        {'type': 'emergencyStopRequest', 'stop': True},
    )

    bridge.end_ws_session()
    bridge._drain_inbound()

    estop = publishers['estop'].messages[-1]
    assert isinstance(estop, EmergencyStopCommand)
    assert estop.stop
    neutral = publishers['drive'].messages[-1]
    assert isinstance(neutral, DriveCommand)
    assert (neutral.straight, neutral.steer) == (0.0, 0.0)


def test_joint_refresh_cannot_cross_sessions(bridge: MCBridgeNode) -> None:
    """A reconnect race cannot stamp old joint power as a new session."""
    publishers = _record_commands(bridge)
    bridge.begin_ws_session()
    _dispatch(
        bridge,
        {'type': 'jointPowerRequest', 'joint': 'elbow', 'power': 0.5},
    )
    published = len(publishers['joint_power'].messages)

    bridge._command_session.begin()
    bridge._heartbeat_tick()

    assert len(publishers['joint_power'].messages) == published


def test_estop_clear_is_not_refreshed(bridge: MCBridgeNode) -> None:
    """A later rover-side stop cannot be cleared without operator input."""
    publishers = _record_commands(bridge)
    bridge.begin_ws_session()
    _dispatch(
        bridge,
        {'type': 'emergencyStopRequest', 'stop': False},
    )
    published = len(publishers['estop'].messages)

    bridge._heartbeat_tick()

    assert len(publishers['estop'].messages) == published


def test_disconnect_closes_active_camera_streams(
    bridge: MCBridgeNode,
) -> None:
    """A clean disconnect explicitly releases rover camera resources."""
    publishers = _record_commands(bridge)
    bridge.begin_ws_session()
    _dispatch(
        bridge,
        {
            'type': 'cameraStreamOpenRequest',
            'camera': 'mast',
            'fps': 20,
        },
    )

    bridge.end_ws_session()

    closing = publishers['camera_command'].messages[-1]
    assert isinstance(closing, CameraCommand)
    assert closing.action == CameraCommand.STREAM_CLOSE
    assert closing.camera == 'mast'


def test_static_reports_replay_on_reconnect(bridge: MCBridgeNode) -> None:
    """Only retained peripheral state is replayed into a fresh session."""
    packets: list[JsonObject] = []

    def record(packet: JsonObject) -> bool:
        packets.append(dict(packet))
        return True

    bridge.set_outbound_publisher(record)
    peripheral = MountedPeripheralReport()
    peripheral.peripheral = MountedPeripheralReport.SCIENCE
    bridge._mounted_peripheral_callback(peripheral)
    ik = ArmIKEnabledReport()
    ik.enabled = True
    bridge._arm_ik_enabled_callback(ik)
    packets.clear()

    bridge.begin_ws_session()

    assert packets == [
        {
            'type': 'mountedPeripheralReport',
            'peripheral': 'science',
        },
        {'type': 'armIKEnabledReport', 'enabled': True},
    ]
