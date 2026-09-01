"""Integration tests for the Mission Control WebSocket transport."""

import asyncio
from collections.abc import Callable
from http import HTTPStatus
import json

from mc_bridge.websocket_server import SingleControllerWebSocketServer
import pytest
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, InvalidStatus


def _uri(
    server: SingleControllerWebSocketServer,
    path: str = '/mission-control',
) -> str:
    return f'ws://127.0.0.1:{server.bound_port}{path}'


def _recorder(
    messages: list[dict[str, object]],
) -> Callable[[dict[str, object]], bool]:
    def record(message: dict[str, object]) -> bool:
        messages.append(message)
        return True

    return record


async def _wait_until(
    predicate: Callable[[], bool],
    timeout: float = 1.0,
) -> None:
    async def poll() -> None:
        while not predicate():
            await asyncio.sleep(0)

    await asyncio.wait_for(poll(), timeout)


def test_valid_packet_is_delivered() -> None:
    """A valid JSON object reaches the configured message handler."""
    received: list[dict[str, object]] = []
    packet: dict[str, object] = {
        'type': 'driveRequest',
        'straight': 0.25,
        'steer': -0.5,
    }

    async def scenario() -> None:
        server = SingleControllerWebSocketServer(_recorder(received), port=0)
        async with server:
            async with connect(_uri(server)) as websocket:
                await websocket.send(json.dumps(packet))
                await _wait_until(lambda: received == [packet])

    asyncio.run(scenario())


def test_wrong_path_is_rejected() -> None:
    """The server only accepts Mission Control's configured path."""
    async def scenario() -> None:
        server = SingleControllerWebSocketServer(lambda _: True, port=0)
        async with server:
            with pytest.raises(InvalidStatus) as raised:
                await connect(_uri(server, '/wrong-path'))
            assert raised.value.response.status_code == HTTPStatus.NOT_FOUND

    asyncio.run(scenario())


def test_second_controller_is_rejected_without_disrupting_first() -> None:
    """Only one controller is accepted, and the first remains usable."""
    received: list[dict[str, object]] = []
    packet: dict[str, object] = {
        'type': 'driveRequest',
        'straight': 1.0,
        'steer': 0.0,
    }

    async def scenario() -> None:
        server = SingleControllerWebSocketServer(_recorder(received), port=0)
        async with server:
            first = await connect(_uri(server))
            try:
                await _wait_until(lambda: server.controller_connected)
                with pytest.raises(InvalidStatus) as raised:
                    await connect(_uri(server))
                assert raised.value.response.status_code == HTTPStatus.CONFLICT

                await first.send(json.dumps(packet))
                await _wait_until(lambda: received == [packet])
            finally:
                await first.close()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    ('payload', 'expected_code'),
    [
        ('not-json', 1007),
        ('{"type":"driveRequest","straight":2,"steer":0}', 1007),
        ('{"type":"roverPositionReport"}', 1007),
        (b'not-text', 1003),
    ],
)
def test_invalid_input_closes_connection(
    payload: str | bytes,
    expected_code: int,
) -> None:
    """Malformed JSON and binary messages close the controller connection."""
    async def scenario() -> None:
        server = SingleControllerWebSocketServer(lambda _: True, port=0)
        async with server:
            websocket = await connect(_uri(server))
            try:
                await websocket.send(payload)
                with pytest.raises(ConnectionClosed) as raised:
                    await websocket.recv()
                assert raised.value.rcvd is not None
                assert raised.value.rcvd.code == expected_code
            finally:
                await websocket.close()

    asyncio.run(scenario())


def test_outbound_packet_is_published_from_another_thread() -> None:
    """A ROS-thread publication reaches the active controller."""
    packet: dict[str, object] = {
        'type': 'driveStateReport',
        'left': 0.5,
        'right': 0.75,
    }

    async def scenario() -> None:
        server = SingleControllerWebSocketServer(lambda _: True, port=0)
        async with server:
            async with connect(_uri(server)) as websocket:
                await _wait_until(lambda: server.controller_connected)
                accepted = await asyncio.to_thread(
                    server.publish_from_thread,
                    packet,
                )
                assert accepted
                raw_message = await asyncio.wait_for(websocket.recv(), 1.0)
                assert json.loads(raw_message) == packet

    asyncio.run(scenario())


def test_connect_handler_can_publish_retained_state() -> None:
    """The connection mailbox exists before static state is replayed."""
    async def scenario() -> None:
        server: SingleControllerWebSocketServer

        def connected() -> None:
            assert server.publish_from_thread(
                {'type': 'mountedPeripheralReport', 'peripheral': 'arm'},
            )

        server = SingleControllerWebSocketServer(
            lambda _: True,
            port=0,
            connect_handler=connected,
        )
        async with server:
            async with connect(_uri(server)) as websocket:
                raw_message = await asyncio.wait_for(websocket.recv(), 1.0)
                assert json.loads(raw_message) == {
                    'type': 'mountedPeripheralReport',
                    'peripheral': 'arm',
                }

    asyncio.run(scenario())


def test_failed_connect_handler_runs_session_cleanup() -> None:
    """Partial controller setup cannot leave a command session active."""
    events: list[str] = []

    def fail_setup() -> None:
        events.append('connect')
        raise RuntimeError('setup failed')

    async def scenario() -> None:
        server = SingleControllerWebSocketServer(
            lambda _: True,
            port=0,
            connect_handler=fail_setup,
            disconnect_handler=lambda: events.append('disconnect'),
        )
        async with server:
            websocket = await connect(_uri(server))
            await websocket.wait_closed()
            assert websocket.close_code == 1011

    asyncio.run(scenario())
    assert events == ['connect', 'disconnect']


def test_disconnect_handler_runs_before_accepting_a_replacement() -> None:
    """A closed session can clear its pending ROS input before reconnect."""
    lifecycle: list[str] = []

    async def scenario() -> None:
        server = SingleControllerWebSocketServer(
            lambda _: True,
            port=0,
            connect_handler=lambda: lifecycle.append('connected'),
            disconnect_handler=lambda: lifecycle.append('disconnected'),
        )
        async with server:
            websocket = await connect(_uri(server))
            await _wait_until(lambda: server.controller_connected)
            assert lifecycle == ['connected']
            await websocket.close()
            await _wait_until(
                lambda: lifecycle == ['connected', 'disconnected'],
            )
            assert not server.controller_connected

    asyncio.run(scenario())


def test_outbound_burst_is_bounded() -> None:
    """A burst keeps only the newest value for each bounded stream."""
    async def scenario() -> None:
        server = SingleControllerWebSocketServer(
            lambda _: True,
            port=0,
            outbound_capacity=2,
        )
        async with server:
            async with connect(_uri(server)) as websocket:
                await _wait_until(lambda: server.controller_connected)

                for packet_type in ('first', 'second', 'third'):
                    assert server.publish_from_thread(
                        {'type': packet_type, 'sequence': packet_type},
                    )

                messages = [
                    json.loads(await asyncio.wait_for(websocket.recv(), 1.0))
                    for _ in range(2)
                ]
                assert [message['type'] for message in messages] == [
                    'second',
                    'third',
                ]
                assert server.dropped_outbound_messages == 1

    asyncio.run(scenario())


def test_outbound_state_is_coalesced_by_identity() -> None:
    """Slow clients receive the latest state for each reported entity."""
    async def scenario() -> None:
        server = SingleControllerWebSocketServer(lambda _: True, port=0)
        async with server:
            async with connect(_uri(server)) as websocket:
                await _wait_until(lambda: server.controller_connected)

                for position in range(3):
                    assert server.publish_from_thread(
                        {
                            'type': 'jointPositionReport',
                            'joint': 'elbow',
                            'position': position,
                        },
                    )

                message = json.loads(
                    await asyncio.wait_for(websocket.recv(), 1.0),
                )
                assert message['position'] == 2
                assert server.dropped_outbound_messages == 2

    asyncio.run(scenario())


def test_outbound_packets_are_never_replayed_after_reconnect() -> None:
    """Disconnected or prior-session packets never reach a new controller."""
    stale_packet: dict[str, object] = {'type': 'state', 'sequence': 1}
    fresh_packet: dict[str, object] = {'type': 'state', 'sequence': 2}

    async def scenario() -> None:
        server = SingleControllerWebSocketServer(lambda _: True, port=0)
        async with server:
            first = await connect(_uri(server))
            await _wait_until(lambda: server.controller_connected)
            assert server.publish_from_thread(stale_packet)
            await first.close()
            await _wait_until(lambda: not server.controller_connected)

            assert not server.publish_from_thread(stale_packet)

            async with connect(_uri(server)) as second:
                await _wait_until(lambda: server.controller_connected)
                with pytest.raises(asyncio.TimeoutError):
                    await asyncio.wait_for(second.recv(), 0.05)

                assert server.publish_from_thread(fresh_packet)
                raw_message = await asyncio.wait_for(second.recv(), 1.0)
                assert json.loads(raw_message) == fresh_packet

    asyncio.run(scenario())
