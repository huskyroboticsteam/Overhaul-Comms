import asyncio
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from mc_bridge_pkg.bridge_node import MCBridgeNode, ws_handler
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ws_loop():
    # Stands in for the websocket event loop that bridge_node runs in main_async
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=5)
    loop.close()


def spin_until(executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, 'Timed out waiting for condition'
        executor.spin_once(timeout_sec=0.1)


def test_drive_request_published_to_ros(ros, ws_loop):
    bridge = MCBridgeNode(ws_loop, asyncio.Queue())
    listener = Node('test_listener')
    received = []
    listener.create_subscription(String, '/mc/drive/cmd', received.append, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(listener)
    spin_until(executor, lambda: bridge.drive_pub.get_subscription_count() > 0)

    request = {'type': 'driveRequest', 'speed': 1.0}
    bridge.handle_ws_message(request)
    spin_until(executor, lambda: received)

    assert json.loads(received[0].data) == request


def test_drive_state_queued_for_mission_control(ros, ws_loop):
    out_queue = asyncio.Queue()
    bridge = MCBridgeNode(ws_loop, out_queue)
    talker = Node('test_talker')
    pub = talker.create_publisher(String, '/mc/drive/state', 10)

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(talker)
    spin_until(executor, lambda: pub.get_subscription_count() > 0)

    pub.publish(String(data='idle'))
    spin_until(executor, lambda: not out_queue.empty())

    payload = asyncio.run_coroutine_threadsafe(out_queue.get(), ws_loop).result(timeout=5)
    assert payload == {'type': 'driveStateReport', 'data': 'idle'}


class FakeWebSocket:
    """Mission control connection that closes cleanly without sending anything."""

    def __init__(self):
        self.request = SimpleNamespace(path='/mission-control')
        self.sent = []

    def __aiter__(self):
        return self

    async def __anext__(self):
        raise StopAsyncIteration

    async def send(self, data):
        self.sent.append(data)


def test_clean_disconnect_leaves_queue_for_next_connection():
    async def run():
        node = MagicMock(ws_clients=set(), out_queue=asyncio.Queue())
        await asyncio.wait_for(ws_handler(FakeWebSocket(), node), timeout=2)

        # A leftover producer from the closed connection would consume this
        await node.out_queue.put({'type': 'driveStateReport', 'data': 'idle'})
        await asyncio.sleep(0.1)

        assert node.out_queue.qsize() == 1
        assert not node.ws_clients

    asyncio.run(run())
