"""ROS 2 adapter for the rover-side command safety gate."""

from __future__ import annotations

import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rover_interfaces.msg import (
    CameraCommand,
    CameraStreamState,
    CameraStreamTarget,
    CommandHeartbeat,
    CommandSessionStart,
    DriveCommand,
    EmergencyStopCommand,
    JointPowerCommand,
    JointPowerTarget,
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
        self.declare_parameter('ordering_hold_sec', 0.25)
        self.declare_parameter('estop_latch_path', '')
        self.declare_parameter('check_period_sec', 0.05)
        self.declare_parameter('topics.heartbeat', 'comms/heartbeat')
        self.declare_parameter('topics.session_start', 'comms/session_start')
        self.declare_parameter('topics.drive', 'comms/drive')
        self.declare_parameter('topics.tank_drive', 'comms/tank_drive')
        self.declare_parameter('topics.joint_power', 'comms/joint_power')
        self.declare_parameter('topics.camera_command', 'comms/camera')
        self.declare_parameter('topics.emergency_stop', 'comms/emergency_stop')
        self.declare_parameter('topics.safe_wheels', 'drive/safe_wheels')
        self.declare_parameter(
            'topics.safe_joint_power',
            'arm/safe_joint_power',
        )
        self.declare_parameter('topics.safety_state', 'safety/state')
        self.declare_parameter(
            'topics.safe_camera_capture',
            'camera/safe_capture',
        )
        self.declare_parameter(
            'topics.safe_camera_stream',
            'camera/safe_stream',
        )
        self.declare_parameter('camera_names', ['mast', 'hand', 'wrist'])

        self._estop_latch_path = self._optional_path('estop_latch_path')
        self._gate = SafetyGate(
            self._positive_number('heartbeat_timeout_sec'),
            self._positive_number('command_timeout_sec'),
            emergency_stop_latched=self._load_estop_latch(),
        )
        self._ordering_hold_sec = self._positive_number('ordering_hold_sec')
        self._camera_names = self._string_list('camera_names')
        self._active_camera_streams: dict[str, tuple[int, int, int]] = {}
        self._pending_camera_commands: dict[
            tuple[str, str],
            tuple[CameraCommand, float],
        ] = {}
        self._pending_estop_clear: (
            tuple[EmergencyStopCommand, float] | None
        ) = None
        commands = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        estop = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        safe_capture = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        safe_stream = QoSProfile(
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
        self._joint_power_publisher = self.create_publisher(
            JointPowerTarget,
            self._topic('topics.safe_joint_power'),
            commands,
        )
        self._camera_capture_publisher = self.create_publisher(
            CameraCommand,
            self._topic('topics.safe_camera_capture'),
            safe_capture,
        )
        self._camera_stream_publisher = self.create_publisher(
            CameraStreamState,
            self._topic('topics.safe_camera_stream'),
            safe_stream,
        )
        self._subscriptions = [
            self.create_subscription(
                CommandSessionStart,
                self._topic('topics.session_start'),
                self._session_start_callback,
                commands,
            ),
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
                JointPowerCommand,
                self._topic('topics.joint_power'),
                self._joint_power_callback,
                commands,
            ),
            self.create_subscription(
                CameraCommand,
                self._topic('topics.camera_command'),
                self._camera_callback,
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
        self._close_camera_streams(self._camera_names)
        self._publish(self._gate.snapshot)
        self.get_logger().info('Rover command watchdog initialized')

    def _session_start_callback(self, message: CommandSessionStart) -> None:
        before = self._gate.snapshot
        self._gate.begin_session(message.session_id, time.monotonic())
        self._publish_if_changed(before)

    def _heartbeat_callback(self, message: CommandHeartbeat) -> None:
        before = self._gate.snapshot
        now = time.monotonic()
        accepted = self._gate.receive_heartbeat(
            message.session_id,
            message.sequence,
            now,
        )
        self._publish_if_changed(before)
        if accepted:
            self._release_pending_commands(message.session_id, now)

    def _drive_callback(self, message: DriveCommand) -> None:
        before = self._gate.snapshot
        accepted = self._gate.receive_drive(
            message.session_id,
            message.sequence,
            message.straight,
            message.steer,
            time.monotonic(),
        )
        changed = self._publish_if_changed(before)
        if accepted and not changed:
            self._publish_wheels(self._gate.snapshot)

    def _tank_drive_callback(self, message: TankDriveCommand) -> None:
        before = self._gate.snapshot
        accepted = self._gate.receive_tank_drive(
            message.session_id,
            message.sequence,
            message.left,
            message.right,
            time.monotonic(),
        )
        changed = self._publish_if_changed(before)
        if accepted and not changed:
            self._publish_wheels(self._gate.snapshot)

    def _joint_power_callback(self, message: JointPowerCommand) -> None:
        before = self._gate.snapshot
        accepted = self._gate.receive_joint_power(
            message.session_id,
            message.sequence,
            message.joint,
            message.power,
            time.monotonic(),
        )
        changed = self._publish_if_changed(before)
        if accepted and not changed:
            self._publish_joint_power(message.joint, message.power)

    def _camera_callback(self, message: CameraCommand) -> None:
        if (
            message.camera not in self._camera_names
            or message.action
            not in (
                CameraCommand.CAPTURE,
                CameraCommand.STREAM_OPEN,
                CameraCommand.STREAM_CLOSE,
            )
            or (
                message.action == CameraCommand.STREAM_OPEN
                and not 0 < message.fps <= 60
            )
        ):
            return

        snapshot = self._gate.snapshot
        if (
            message.session_id != snapshot.active_session_id
            or snapshot.heartbeat_timed_out
        ):
            kind = (
                'capture'
                if message.action == CameraCommand.CAPTURE
                else 'stream'
            )
            self._pending_camera_commands[(message.camera, kind)] = (
                message,
                time.monotonic() + self._ordering_hold_sec,
            )
            return
        self._publish_camera_command(message)

    def _publish_camera_command(self, message: CameraCommand) -> None:
        if not self._gate.accept_command(
            message.session_id,
            message.sequence,
            f'camera:{message.camera}',
        ):
            return
        if message.action == CameraCommand.STREAM_OPEN:
            self._active_camera_streams[message.camera] = (
                message.session_id,
                message.sequence,
                message.fps,
            )
        elif message.action == CameraCommand.STREAM_CLOSE:
            self._active_camera_streams.pop(message.camera, None)
        if message.action == CameraCommand.CAPTURE:
            self._camera_capture_publisher.publish(message)
        else:
            self._publish_camera_stream_snapshot()

    def _release_pending_commands(self, session_id: int, now: float) -> None:
        ready: list[CameraCommand | EmergencyStopCommand] = []
        for key, pending in tuple(self._pending_camera_commands.items()):
            message, deadline = pending
            if now > deadline:
                del self._pending_camera_commands[key]
            elif message.session_id == session_id:
                del self._pending_camera_commands[key]
                ready.append(message)

        pending_clear = self._pending_estop_clear
        if pending_clear is not None:
            message, deadline = pending_clear
            if now > deadline:
                self._pending_estop_clear = None
            elif message.session_id == session_id:
                self._pending_estop_clear = None
                ready.append(message)

        for message in sorted(ready, key=lambda item: item.sequence):
            if isinstance(message, EmergencyStopCommand):
                self._apply_emergency_stop(message)
            else:
                self._publish_camera_command(message)

    def _emergency_stop_callback(self, message: EmergencyStopCommand) -> None:
        if message.stop:
            self._pending_estop_clear = None
            self._apply_emergency_stop(message)
            return

        snapshot = self._gate.snapshot
        if (
            message.session_id != snapshot.active_session_id
            or snapshot.heartbeat_timed_out
        ):
            self._pending_estop_clear = (
                message,
                time.monotonic() + self._ordering_hold_sec,
            )
            return
        self._apply_emergency_stop(message)

    def _apply_emergency_stop(self, message: EmergencyStopCommand) -> None:
        before = self._gate.snapshot
        accepted = self._gate.receive_emergency_stop(
            message.session_id,
            message.sequence,
            message.stop,
        )
        after = self._gate.snapshot
        if (
            accepted
            and after.emergency_stop_latched
            != before.emergency_stop_latched
        ):
            if after.emergency_stop_latched:
                self._publish_if_changed(before)
                self._store_estop_latch(True)
            else:
                self._store_estop_latch(False)
                self._publish_if_changed(before)
            return
        self._publish_if_changed(before)

    def _check_timeouts(self) -> None:
        before = self._gate.snapshot
        now = time.monotonic()
        self._gate.check_timeouts(now)
        self._release_pending_commands(0, now)
        if not self._publish_if_changed(before):
            self._publish_state(self._gate.snapshot)

    def _publish_if_changed(self, before: SafetySnapshot) -> bool:
        after = self._gate.snapshot
        if after == before:
            return False
        if (
            after.active_session_id != before.active_session_id
            or after.emergency_stop_latched
            or after.heartbeat_timed_out
        ):
            self._close_camera_streams(
                tuple(self._active_camera_streams),
            )
        self._publish(after, before)
        return True

    def _close_camera_streams(self, cameras: tuple[str, ...]) -> None:
        for camera in cameras:
            self._active_camera_streams.pop(camera, None)
        if cameras:
            self._publish_camera_stream_snapshot()

    def _publish_camera_stream_snapshot(self) -> None:
        state = CameraStreamState()
        state.session_id = self._gate.snapshot.active_session_id
        targets: list[CameraStreamTarget] = []
        for camera in self._camera_names:
            active = self._active_camera_streams.get(camera)
            target = CameraStreamTarget()
            target.camera = camera
            if active is not None:
                target.open = True
                target.fps = active[2]
                target.request_sequence = active[1]
            targets.append(target)
        state.cameras = targets
        self._camera_stream_publisher.publish(state)

    def _publish(
        self,
        snapshot: SafetySnapshot,
        previous: SafetySnapshot | None = None,
    ) -> None:
        self._publish_wheels(snapshot)
        self._publish_state(snapshot)

        before = {
            target.joint: target.power
            for target in previous.joint_powers
        } if previous is not None else {}
        after = {
            target.joint: target.power
            for target in snapshot.joint_powers
        }
        for joint in sorted(before.keys() | after.keys()):
            if before.get(joint) == after.get(joint):
                continue
            self._publish_joint_power(joint, after.get(joint, 0.0))

    def _publish_wheels(self, snapshot: SafetySnapshot) -> None:
        wheels = WheelCommand()
        wheels.left = snapshot.wheels.left
        wheels.right = snapshot.wheels.right
        self._wheel_publisher.publish(wheels)

    def _publish_state(self, snapshot: SafetySnapshot) -> None:
        state = SafetyState()
        state.active_session_id = snapshot.active_session_id
        state.emergency_stop_latched = snapshot.emergency_stop_latched
        state.heartbeat_timed_out = snapshot.heartbeat_timed_out
        state.command_timed_out = snapshot.command_timed_out
        self._state_publisher.publish(state)

    def _publish_joint_power(self, joint: str, power: float) -> None:
        target = JointPowerTarget()
        target.joint = joint
        target.power = power
        self._joint_power_publisher.publish(target)

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

    def _string_list(self, name: str) -> tuple[str, ...]:
        value = self.get_parameter(name).value
        if (
            not isinstance(value, (list, tuple))
            or not value
            or any(
                not isinstance(item, str)
                or not item
                or len(item) > 16
                for item in value
            )
            or len(set(value)) != len(value)
            or len(value) > 3
        ):
            raise ValueError(f'Parameter {name} has invalid camera names')
        return tuple(value)

    def _optional_path(self, name: str) -> Path | None:
        value = self.get_parameter(name).value
        if not isinstance(value, str):
            raise ValueError(f'Parameter {name} must be a string')
        return Path(value).expanduser() if value else None

    def _load_estop_latch(self) -> bool:
        path = self._estop_latch_path
        if path is None:
            return False
        try:
            path.read_bytes()
        except FileNotFoundError:
            return False
        except OSError as error:
            raise RuntimeError(
                f'Could not read emergency-stop latch: {path}'
            ) from error
        return True

    def _store_estop_latch(self, latched: bool) -> None:
        path = self._estop_latch_path
        if path is None:
            return
        try:
            if not latched:
                path.unlink(missing_ok=True)
                return
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_name(f'.{path.name}.tmp')
            temporary.write_text('latched\n', encoding='utf-8')
            temporary.replace(path)
        except OSError as error:
            raise RuntimeError(
                f'Could not persist emergency-stop latch: {path}'
            ) from error


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
