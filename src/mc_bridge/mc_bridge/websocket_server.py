"""Single-controller WebSocket transport for Mission Control."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from collections import OrderedDict
from http import HTTPStatus
import logging
import threading
from types import TracebackType
from typing import TypeAlias, cast

from websockets.asyncio.server import Server, ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from mc_bridge.packet import (
    JsonObject,
    PacketValidationError,
    decode_packet,
    encode_packet,
    validate_packet,
)

MessageHandler: TypeAlias = Callable[[JsonObject], bool]
ConnectHandler: TypeAlias = Callable[[], None]
DisconnectHandler: TypeAlias = Callable[[], None]
PacketKey: TypeAlias = tuple[str, ...]


class _MailboxClosed(Exception):
    """Signal that a connection-scoped outbound mailbox was closed."""


class _LatestValueMailbox:
    """Move bounded latest-value data into one asyncio session."""

    def __init__(self, capacity: int) -> None:
        self._capacity = capacity
        self._loop = asyncio.get_running_loop()
        self._ready = asyncio.Event()
        self._items: OrderedDict[PacketKey, str] = OrderedDict()
        self._lock = threading.Lock()
        self._notified = False
        self._closed = False
        self._dropped = 0

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped

    def offer(self, key: PacketKey, message: str) -> bool:
        """Retain only the newest value for each bounded packet stream."""
        with self._lock:
            if self._closed:
                return False

            if key in self._items:
                del self._items[key]
                self._dropped += 1
            elif len(self._items) == self._capacity:
                self._items.popitem(last=False)
                self._dropped += 1
            self._items[key] = message

            if self._notified:
                return True
            self._notified = True
            try:
                self._loop.call_soon_threadsafe(self._ready.set)
            except RuntimeError:
                self._notified = False
                self._closed = True
                self._items.clear()
                return False
            return True

    async def get(self) -> str:
        """Wait for and remove the oldest retained message."""
        while True:
            await self._ready.wait()
            with self._lock:
                if self._closed:
                    raise _MailboxClosed
                if self._items:
                    _, message = self._items.popitem(last=False)
                    if not self._items:
                        self._ready.clear()
                        self._notified = False
                    return message
                self._ready.clear()
                self._notified = False

    def close(self) -> None:
        """Discard pending messages and wake the consumer."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._items.clear()
            if self._notified:
                return
            self._notified = True
            try:
                self._loop.call_soon_threadsafe(self._ready.set)
            except RuntimeError:
                pass


class SingleControllerWebSocketServer:
    """Accept one Mission Control client and drop data between sessions."""

    def __init__(
        self,
        message_handler: MessageHandler,
        *,
        host: str = '127.0.0.1',
        port: int = 3001,
        path: str = '/mission-control',
        outbound_capacity: int = 64,
        ping_interval: float = 1.0,
        ping_timeout: float = 1.0,
        connect_handler: ConnectHandler | None = None,
        disconnect_handler: DisconnectHandler | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        """Configure the server without binding its listening socket."""
        if not path.startswith('/'):
            raise ValueError('WebSocket path must start with "/"')
        if outbound_capacity < 1:
            raise ValueError('Outbound capacity must be positive')
        if ping_interval <= 0.0 or ping_timeout <= 0.0:
            raise ValueError('WebSocket ping settings must be positive')

        self._message_handler = message_handler
        self._host = host
        self._port = port
        self._path = path
        self._outbound_capacity = outbound_capacity
        self._ping_interval = ping_interval
        self._ping_timeout = ping_timeout
        self._connect_handler = connect_handler or (lambda: None)
        self._disconnect_handler = disconnect_handler or (lambda: None)
        self._logger = logger or logging.getLogger(__name__)

        self._server: Server | None = None
        self._controller: ServerConnection | None = None
        self._thread_state_lock = threading.Lock()
        self._mailbox: _LatestValueMailbox | None = None
        self._dropped_outbound_messages = 0

    @property
    def bound_port(self) -> int:
        """Return the bound TCP port after the server starts."""
        server = self._server
        if server is None:
            raise RuntimeError('WebSocket server is not running')
        listening_socket = next(iter(server.sockets), None)
        if listening_socket is None:
            raise RuntimeError('WebSocket server has no listening socket')
        return int(listening_socket.getsockname()[1])

    @property
    def controller_connected(self) -> bool:
        """Return whether a controlling client is active."""
        return self._controller is not None

    @property
    def dropped_outbound_messages(self) -> int:
        """Return the number of messages evicted by outbound backpressure."""
        with self._thread_state_lock:
            active_dropped = self._mailbox.dropped if self._mailbox else 0
            return self._dropped_outbound_messages + active_dropped

    async def __aenter__(self) -> SingleControllerWebSocketServer:
        """Start accepting WebSocket connections."""
        if self._server is not None:
            raise RuntimeError('WebSocket server is already running')

        self._server = await serve(
            self._handle_connection,
            self._host,
            self._port,
            process_request=self._process_request,
            max_queue=16,
            ping_interval=self._ping_interval,
            ping_timeout=self._ping_timeout,
        )
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Stop accepting connections and close the active controller."""
        server = self._server
        self._server = None
        if server is not None:
            server.close()
            await server.wait_closed()

    def publish_from_thread(self, payload: JsonObject) -> bool:
        """Offer a ROS-originated payload to the current controller."""
        with self._thread_state_lock:
            mailbox = self._mailbox

        if mailbox is None:
            return False

        packet = dict(payload)
        try:
            message = encode_packet(packet)
        except PacketValidationError:
            self._logger.exception('Could not serialize outbound packet')
            return False
        return mailbox.offer(_packet_key(packet), message)

    def _process_request(
        self,
        connection: ServerConnection,
        request: Request,
    ) -> Response | None:
        if request.path != self._path:
            return connection.respond(HTTPStatus.NOT_FOUND, 'Not Found\n')
        if self._controller is not None:
            return connection.respond(
                HTTPStatus.CONFLICT,
                'A Mission Control client is already connected\n',
            )
        return None

    async def _handle_connection(self, websocket: ServerConnection) -> None:
        # Close the small race between concurrent handshakes.
        if self._controller is not None:
            await websocket.close(
                code=1013,
                reason='Controller already connected',
            )
            return

        mailbox = _LatestValueMailbox(self._outbound_capacity)
        self._controller = websocket
        with self._thread_state_lock:
            self._mailbox = mailbox

        try:
            self._connect_handler()
        except Exception:
            self._logger.exception('WebSocket connect handler failed')
            try:
                self._disconnect_handler()
            except Exception:
                self._logger.exception(
                    'WebSocket setup cleanup failed',
                )
            mailbox.close()
            with self._thread_state_lock:
                if self._mailbox is mailbox:
                    self._mailbox = None
                self._dropped_outbound_messages += mailbox.dropped
            if self._controller is websocket:
                self._controller = None
            await websocket.close(code=1011, reason='Controller setup failed')
            return

        self._logger.info('Mission Control connected')
        receiver = asyncio.create_task(
            self._receive_messages(websocket),
            name='mission-control-receiver',
        )
        sender = asyncio.create_task(
            self._send_messages(websocket, mailbox),
            name='mission-control-sender',
        )
        tasks = {receiver, sender}

        try:
            done, _ = await asyncio.wait(
                tasks,
                return_when=asyncio.FIRST_COMPLETED,
            )
            for task in done:
                try:
                    task.result()
                except ConnectionClosed:
                    pass
        finally:
            for task in tasks:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

            mailbox.close()
            with self._thread_state_lock:
                if self._mailbox is mailbox:
                    self._mailbox = None
                self._dropped_outbound_messages += mailbox.dropped
            if self._controller is websocket:
                try:
                    self._disconnect_handler()
                except Exception:
                    self._logger.exception(
                        'WebSocket disconnect handler failed',
                    )
                self._controller = None
            self._logger.info('Mission Control disconnected')

    async def _receive_messages(self, websocket: ServerConnection) -> None:
        async for raw_message in websocket:
            if not isinstance(raw_message, str):
                await websocket.close(
                    code=1003,
                    reason='Text messages required',
                )
                return

            try:
                message = decode_packet(raw_message)
                validate_packet(message, direction='request')
            except PacketValidationError:
                await websocket.close(code=1007, reason='Invalid packet')
                return

            if not self._message_handler(message):
                await websocket.close(
                    code=1013,
                    reason='ROS input queue is full',
                )
                return

    async def _send_messages(
        self,
        websocket: ServerConnection,
        mailbox: _LatestValueMailbox,
    ) -> None:
        try:
            while True:
                await websocket.send(await mailbox.get())
        except _MailboxClosed:
            return


def _packet_key(packet: JsonObject) -> PacketKey:
    """Identify state streams that should replace their older value."""
    packet_type = cast(str, packet['type'])
    for field_name in ('camera', 'joint', 'servo'):
        identity = packet.get(field_name)
        if isinstance(identity, str):
            return packet_type, field_name, identity
    return (packet_type,)
