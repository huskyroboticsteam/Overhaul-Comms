"""Tests for the directional Zenoh ROS topic allowlists."""

import json
from pathlib import Path
import re


_COMMAND_TOPICS = {
    '/comms/drive',
    '/comms/tank_drive',
    '/comms/emergency_stop',
    '/comms/heartbeat',
    '/comms/session_start',
    '/comms/operation_mode',
    '/comms/joint_power',
    '/comms/joint_position',
    '/comms/arm_ik',
    '/comms/servo_position',
    '/comms/stepper_turn_angle',
    '/comms/waypoint_nav',
    '/comms/camera',
}
_REPORT_TOPICS = {
    '/telemetry/rover_position',
    '/telemetry/joint_position',
    '/telemetry/servo_position',
    '/telemetry/peripheral',
    '/telemetry/arm_ik',
    '/camera/frame',
    '/camera/stream',
    '/safety/state',
}


def _allowlist(host: str, kind: str) -> list[str]:
    path = (
        Path(__file__).resolve().parents[3]
        / 'config'
        / 'zenoh'
        / f'{host}.json5'
    )
    config = json.loads(path.read_text(encoding='utf-8'))
    return config['plugins']['ros2dds']['allow'][kind]


def _matches(topic: str, patterns: list[str]) -> bool:
    return any(re.fullmatch(pattern, topic) for pattern in patterns)


def test_allowlists_are_exact_and_directional() -> None:
    """Only declared bridge topics cross in their intended direction."""
    laptop_publishers = _allowlist('laptop', 'publishers')
    laptop_subscribers = _allowlist('laptop', 'subscribers')
    rover_publishers = _allowlist('rover', 'publishers')
    rover_subscribers = _allowlist('rover', 'subscribers')

    assert laptop_publishers == rover_subscribers
    assert laptop_subscribers == rover_publishers
    assert all(_matches(topic, laptop_publishers) for topic in _COMMAND_TOPICS)
    assert all(_matches(topic, laptop_subscribers) for topic in _REPORT_TOPICS)
    assert not _matches('/comms/blocked', laptop_publishers)
    assert not _matches('/camera/safe_capture', laptop_subscribers)
    assert not _matches('/camera/safe_stream', laptop_subscribers)
