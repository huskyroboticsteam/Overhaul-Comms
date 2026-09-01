"""ROS-level tests for rover-local camera safety leases."""

from __future__ import annotations

from collections.abc import Iterator

import pytest

rclpy = pytest.importorskip('rclpy')

from rover_interfaces.msg import (  # noqa: E402
    CameraCommand,
    CommandHeartbeat,
    CommandSessionStart,
    DriveCommand,
    EmergencyStopCommand,
    JointPowerCommand,
)

from rover_safety.watchdog_node import RoverWatchdogNode  # noqa: E402


class _PublisherRecorder:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


def _has_stream_state(
    messages: list[object],
    camera: str,
    open_: bool,
) -> bool:
    return any(
        target.camera == camera and target.open is open_
        for message in messages
        for target in message.cameras
    )


def _start_session(watchdog: RoverWatchdogNode) -> None:
    start = CommandSessionStart()
    start.session_id = 10
    watchdog._session_start_callback(start)


@pytest.fixture
def watchdog() -> Iterator[RoverWatchdogNode]:
    """Create and reliably destroy one initialized watchdog node."""
    rclpy.init()
    node = RoverWatchdogNode()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_camera_stream_closes_when_heartbeat_expires(
    watchdog: RoverWatchdogNode,
) -> None:
    """A rover-local lease closes video without laptop assistance."""
    publisher = _PublisherRecorder()
    watchdog._camera_stream_publisher = publisher

    _start_session(watchdog)
    heartbeat = CommandHeartbeat()
    heartbeat.session_id = 10
    heartbeat.sequence = 1
    watchdog._heartbeat_callback(heartbeat)

    opening = CameraCommand()
    opening.session_id = 10
    opening.sequence = 2
    opening.action = CameraCommand.STREAM_OPEN
    opening.camera = 'mast'
    opening.fps = 20
    watchdog._camera_callback(opening)
    assert _has_stream_state(publisher.messages, 'mast', True)

    before = watchdog._gate.snapshot
    watchdog._gate.check_timeouts(float('inf'))
    watchdog._publish_if_changed(before)

    assert _has_stream_state(publisher.messages, 'mast', False)


def test_camera_command_waits_for_its_heartbeat(
    watchdog: RoverWatchdogNode,
) -> None:
    """Inter-topic reordering cannot lose a fresh one-shot command."""
    captures = _PublisherRecorder()
    streams = _PublisherRecorder()
    watchdog._camera_capture_publisher = captures
    watchdog._camera_stream_publisher = streams
    _start_session(watchdog)

    capture = CameraCommand()
    capture.session_id = 10
    capture.sequence = 2
    capture.action = CameraCommand.CAPTURE
    capture.camera = 'mast'
    watchdog._camera_callback(capture)

    opening = CameraCommand()
    opening.session_id = 10
    opening.sequence = 3
    opening.action = CameraCommand.STREAM_OPEN
    opening.camera = 'mast'
    opening.fps = 20
    watchdog._camera_callback(opening)
    assert captures.messages == []
    assert streams.messages == []

    heartbeat = CommandHeartbeat()
    heartbeat.session_id = 10
    heartbeat.sequence = 1
    watchdog._heartbeat_callback(heartbeat)

    assert captures.messages == [capture]
    assert _has_stream_state(streams.messages, 'mast', True)


def test_estop_clear_waits_for_its_heartbeat(
    watchdog: RoverWatchdogNode,
) -> None:
    """Inter-topic reordering cannot lose a fresh e-stop clear."""
    stop = EmergencyStopCommand()
    stop.stop = True
    watchdog._emergency_stop_callback(stop)
    _start_session(watchdog)

    clear = EmergencyStopCommand()
    clear.session_id = 10
    clear.sequence = 2
    watchdog._emergency_stop_callback(clear)
    assert watchdog._gate.snapshot.emergency_stop_latched

    heartbeat = CommandHeartbeat()
    heartbeat.session_id = 10
    heartbeat.sequence = 1
    watchdog._heartbeat_callback(heartbeat)
    assert not watchdog._gate.snapshot.emergency_stop_latched


def test_estop_assertion_cancels_a_pending_clear(
    watchdog: RoverWatchdogNode,
) -> None:
    """A later stop always takes precedence over a reordered clear."""
    initial_stop = EmergencyStopCommand()
    initial_stop.stop = True
    watchdog._emergency_stop_callback(initial_stop)
    _start_session(watchdog)

    clear = EmergencyStopCommand()
    clear.session_id = 10
    clear.sequence = 2
    watchdog._emergency_stop_callback(clear)

    later_stop = EmergencyStopCommand()
    later_stop.session_id = 10
    later_stop.sequence = 3
    later_stop.stop = True
    watchdog._emergency_stop_callback(later_stop)

    heartbeat = CommandHeartbeat()
    heartbeat.session_id = 10
    heartbeat.sequence = 1
    watchdog._heartbeat_callback(heartbeat)
    assert watchdog._gate.snapshot.emergency_stop_latched


def test_identical_safe_commands_are_refreshed(
    watchdog: RoverWatchdogNode,
) -> None:
    """Healthy continuous commands refresh downstream hardware leases."""
    wheels = _PublisherRecorder()
    joints = _PublisherRecorder()
    watchdog._wheel_publisher = wheels
    watchdog._joint_power_publisher = joints

    _start_session(watchdog)
    heartbeat = CommandHeartbeat()
    heartbeat.session_id = 10
    heartbeat.sequence = 1
    watchdog._heartbeat_callback(heartbeat)
    wheels.messages.clear()

    for sequence in (2, 3):
        drive = DriveCommand()
        drive.session_id = 10
        drive.sequence = sequence
        drive.straight = 0.5
        watchdog._drive_callback(drive)

    assert len(wheels.messages) == 2

    for sequence in (4, 5):
        joint = JointPowerCommand()
        joint.session_id = 10
        joint.sequence = sequence
        joint.joint = 'elbow'
        joint.power = 0.25
        watchdog._joint_power_callback(joint)

    assert len(joints.messages) == 2
