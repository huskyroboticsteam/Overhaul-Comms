#!/usr/bin/env python3
"""End-to-end check for the WebSocket, Zenoh, and rover safety path."""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Callable
import json
import math
import os
import socket
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rover_interfaces.msg import (
    CameraCommand,
    CameraStreamState,
    RoverPositionReport,
    SafetyState,
    WheelCommand,
)
from websockets.asyncio.client import ClientConnection, connect
from websockets.exceptions import InvalidHandshake


_FORBIDDEN_TOPIC = 'comms/blocked'
_PROBE_PORT = 39001
_TIMEOUT = 20.0
_POSITION_REPORT = {
    'type': 'roverPositionReport',
    'orientW': 1.0,
    'orientX': 0.0,
    'orientY': 0.0,
    'orientZ': 0.0,
    'lon': -122.0,
    'lat': 47.0,
    'alt': 100.0,
    'recency': 0.125,
}


class _RoverProbe(Node):
    """Collect rover-local output without allowing it over Zenoh."""

    def __init__(self) -> None:
        super().__init__('zenoh_integration_probe')
        self._condition = threading.Condition()
        self._wheels: list[tuple[float, float]] = []
        self._states: list[bool] = []
        self._camera_actions: list[tuple[str, int]] = []
        self._blocked_messages = 0

        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        state = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        telemetry = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        camera_state = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._position_publisher = self.create_publisher(
            RoverPositionReport,
            'telemetry/rover_position',
            telemetry,
        )
        self._position_timer = self.create_timer(
            0.1,
            self._publish_position,
        )
        self._subscriptions = [
            self.create_subscription(
                WheelCommand,
                'drive/safe_wheels',
                self._wheel_callback,
                reliable,
            ),
            self.create_subscription(
                SafetyState,
                'safety/state',
                self._state_callback,
                state,
            ),
            self.create_subscription(
                CameraStreamState,
                'camera/safe_stream',
                self._camera_callback,
                camera_state,
            ),
            self.create_subscription(
                WheelCommand,
                _FORBIDDEN_TOPIC,
                self._blocked_callback,
                reliable,
            ),
        ]

    @property
    def blocked_messages(self) -> int:
        with self._condition:
            return self._blocked_messages

    @property
    def wheel_count(self) -> int:
        with self._condition:
            return len(self._wheels)

    @property
    def camera_count(self) -> int:
        with self._condition:
            return len(self._camera_actions)

    @property
    def state_count(self) -> int:
        with self._condition:
            return len(self._states)

    def wait_for_wheels(
        self,
        left: float,
        right: float,
        *,
        after: int = 0,
        timeout: float = _TIMEOUT,
    ) -> bool:
        return self._wait_for(
            lambda: any(
                math.isclose(actual_left, left, abs_tol=1e-4)
                and math.isclose(actual_right, right, abs_tol=1e-4)
                for actual_left, actual_right in self._wheels[after:]
            ),
            timeout,
        )

    def wait_for_estop(
        self,
        latched: bool,
        *,
        after: int = 0,
        timeout: float = _TIMEOUT,
    ) -> bool:
        return self._wait_for(
            lambda: latched in self._states[after:],
            timeout,
        )

    def wait_for_camera(
        self,
        action: int,
        camera: str,
        *,
        after: int = 0,
        timeout: float = _TIMEOUT,
    ) -> bool:
        return self._wait_for(
            lambda: (camera, action) in self._camera_actions[after:],
            timeout,
        )

    def wheels_are_neutral_since(self, index: int) -> bool:
        with self._condition:
            return all(
                math.isclose(left, 0.0, abs_tol=1e-4)
                and math.isclose(right, 0.0, abs_tol=1e-4)
                for left, right in self._wheels[index:]
            )

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        timeout: float,
    ) -> bool:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def _wheel_callback(self, message: WheelCommand) -> None:
        with self._condition:
            self._wheels.append((message.left, message.right))
            self._condition.notify_all()

    def _state_callback(self, message: SafetyState) -> None:
        with self._condition:
            self._states.append(message.emergency_stop_latched)
            self._condition.notify_all()

    def _camera_callback(self, message: CameraStreamState) -> None:
        with self._condition:
            self._camera_actions.extend(
                (
                    target.camera,
                    CameraCommand.STREAM_OPEN
                    if target.open
                    else CameraCommand.STREAM_CLOSE,
                )
                for target in message.cameras
            )
            self._condition.notify_all()

    def _blocked_callback(self, _: WheelCommand) -> None:
        with self._condition:
            self._blocked_messages += 1
            self._condition.notify_all()

    def _publish_position(self) -> None:
        message = RoverPositionReport()
        message.orientation_w = 1.0
        message.longitude_deg = -122.0
        message.latitude_deg = 47.0
        message.altitude_m = 100.0
        message.recency_sec = 0.125
        self._position_publisher.publish(message)


async def _open_websocket(uri: str) -> ClientConnection:
    deadline = asyncio.get_running_loop().time() + _TIMEOUT
    while True:
        try:
            return await connect(uri, open_timeout=2.0)
        except (InvalidHandshake, OSError, TimeoutError):
            if asyncio.get_running_loop().time() >= deadline:
                raise
            await asyncio.sleep(0.25)


async def _trigger_forbidden_publisher(host: str) -> None:
    deadline = asyncio.get_running_loop().time() + _TIMEOUT
    while True:
        try:
            reader, writer = await asyncio.open_connection(host, _PROBE_PORT)
            break
        except OSError:
            if asyncio.get_running_loop().time() >= deadline:
                raise
            await asyncio.sleep(0.25)

    try:
        writer.write(b'publish\n')
        await writer.drain()
        response = await asyncio.wait_for(reader.readline(), 5.0)
        if response != b'ok\n':
            raise RuntimeError('Forbidden publisher did not acknowledge')
    finally:
        writer.close()
        await writer.wait_closed()


async def _wait_for_position_report(
    websocket: ClientConnection,
) -> None:
    deadline = asyncio.get_running_loop().time() + _TIMEOUT
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0.0:
            raise RuntimeError('Position report did not reach Mission Control')
        try:
            payload = await asyncio.wait_for(
                websocket.recv(),
                min(remaining, 1.0),
            )
        except TimeoutError:
            continue
        packet = json.loads(payload)
        if not isinstance(packet, dict):
            raise RuntimeError('Mission Control received a non-object packet')
        if packet.get('type') != 'roverPositionReport':
            continue
        if packet != _POSITION_REPORT:
            raise RuntimeError(f'Incorrect position report: {packet!r}')
        return


async def _verify(probe: _RoverProbe) -> None:
    uri = os.getenv(
        'MISSION_CONTROL_URI',
        'ws://laptop_zenoh:3001/mission-control',
    )
    websocket = await _open_websocket(uri)
    async with websocket:
        await _wait_for_position_report(websocket)
        before_camera = probe.camera_count
        await websocket.send(json.dumps({
            'type': 'cameraStreamOpenRequest',
            'camera': 'mast',
            'fps': 20,
        }))
        camera_opened = await asyncio.to_thread(
            probe.wait_for_camera,
            CameraCommand.STREAM_OPEN,
            'mast',
            after=before_camera,
        )
        if not camera_opened:
            raise RuntimeError('Camera command did not reach rover lease')

        await websocket.send(json.dumps({
            'type': 'driveRequest',
            'straight': 0.5,
            'steer': -0.25,
        }))
        moved = await asyncio.to_thread(
            probe.wait_for_wheels,
            0.25,
            0.75,
        )
        if not moved:
            raise RuntimeError('Drive command did not reach rover safety')

        before_tank = probe.wheel_count
        await websocket.send(json.dumps({
            'type': 'tankDriveRequest',
            'left': -0.4,
            'right': 0.6,
        }))
        tank_moved = await asyncio.to_thread(
            probe.wait_for_wheels,
            -0.4,
            0.6,
            after=before_tank,
        )
        if not tank_moved:
            raise RuntimeError('Tank drive did not reach rover safety')

        before_disconnect = probe.wheel_count
        before_camera_close = probe.camera_count

    disconnected = await asyncio.to_thread(
        probe.wait_for_wheels,
        0.0,
        0.0,
        after=before_disconnect,
    )
    if not disconnected:
        raise RuntimeError('WebSocket disconnect did not stop rover motion')
    camera_closed = await asyncio.to_thread(
        probe.wait_for_camera,
        CameraCommand.STREAM_CLOSE,
        'mast',
        after=before_camera_close,
    )
    if not camera_closed:
        raise RuntimeError('WebSocket disconnect did not close camera stream')

    websocket = await _open_websocket(uri)
    async with websocket:
        before_reconnect_drive = probe.wheel_count
        await websocket.send(json.dumps({
            'type': 'driveRequest',
            'straight': 0.25,
            'steer': 0.0,
        }))
        moved = await asyncio.to_thread(
            probe.wait_for_wheels,
            0.25,
            0.25,
            after=before_reconnect_drive,
        )
        if not moved:
            raise RuntimeError('Drive did not recover with a fresh session')

        before_estop = probe.wheel_count
        before_estop_state = probe.state_count
        await websocket.send(json.dumps({
            'type': 'emergencyStopRequest',
            'stop': True,
        }))
        stopped, latched = await asyncio.gather(
            asyncio.to_thread(
                probe.wait_for_wheels,
                0.0,
                0.0,
                after=before_estop,
            ),
            asyncio.to_thread(
                probe.wait_for_estop,
                True,
                after=before_estop_state,
            ),
        )
        if not stopped or not latched:
            raise RuntimeError('Emergency stop did not latch and stop motion')

        after_estop = probe.wheel_count
        await websocket.send(json.dumps({
            'type': 'driveRequest',
            'straight': 1.0,
            'steer': 0.0,
        }))
        await asyncio.sleep(0.75)
        if not probe.wheels_are_neutral_since(after_estop):
            raise RuntimeError(
                'Motion resumed while emergency stop was latched',
            )

        before_clear = probe.state_count
        await websocket.send(json.dumps({
            'type': 'emergencyStopRequest',
            'stop': False,
        }))
        cleared = await asyncio.to_thread(
            probe.wait_for_estop,
            False,
            after=before_clear,
        )
        if not cleared:
            raise RuntimeError('Emergency stop did not clear')

        before_recovery = probe.wheel_count
        await websocket.send(json.dumps({
            'type': 'driveRequest',
            'straight': 0.2,
            'steer': 0.0,
        }))
        recovered = await asyncio.to_thread(
            probe.wait_for_wheels,
            0.2,
            0.2,
            after=before_recovery,
        )
        if not recovered:
            raise RuntimeError(
                'Fresh motion did not recover after e-stop clear'
            )

    await asyncio.sleep(1.0)
    await _trigger_forbidden_publisher(
        os.getenv('FORBIDDEN_PUBLISHER_HOST', 'laptop_zenoh'),
    )
    await asyncio.sleep(1.0)
    if probe.blocked_messages:
        raise RuntimeError('Unlisted ROS topic crossed the Zenoh allowlist')


def _run_verification() -> None:
    rclpy.init()
    probe = _RoverProbe()
    executor = SingleThreadedExecutor(context=probe.context)
    executor.add_node(probe)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        asyncio.run(_verify(probe))
        print('Zenoh integration test passed', flush=True)
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(probe)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_forbidden_publisher() -> None:
    rclpy.init()
    node = rclpy.create_node('zenoh_forbidden_publisher')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(WheelCommand, _FORBIDDEN_TOPIC, qos)
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', _PROBE_PORT))
    server.listen()
    try:
        while rclpy.ok():
            connection, _ = server.accept()
            with connection:
                if connection.recv(64).strip() != b'publish':
                    connection.sendall(b'error\n')
                    continue
                for sequence in range(20):
                    message = WheelCommand()
                    message.left = float(sequence)
                    publisher.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(0.05)
                connection.sendall(b'ok\n')
    finally:
        server.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    """Run one side of the integration harness."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'mode',
        choices=('verify', 'forbidden-publisher'),
    )
    mode = parser.parse_args().mode
    if mode == 'verify':
        _run_verification()
    else:
        _run_forbidden_publisher()


if __name__ == '__main__':
    main()
