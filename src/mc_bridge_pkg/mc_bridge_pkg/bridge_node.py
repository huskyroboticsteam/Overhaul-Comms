import asyncio
import json
import logging
import signal
import threading
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import websockets
except Exception:
    websockets = None


class MCBridgeNode(Node):
    """ROS2 node that bridges JSON messages to/from WebSocket clients.

    This is a minimal, extensible skeleton. It uses `std_msgs/String` topics
    carrying JSON payloads for demo purposes; replace with typed ROS msgs
    as your schema matures.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop):
        """
        Initialize MCBridgeNode. Loop is the event loop used for all 
        web socket related communication. Out_queue is created to 
        queue message payloads to the event loop. 

        (i.e. payloads are added on subscriber callbacks to the queue
        so that data can be sent to mission control via web socket)
        
        """
        super().__init__('mc_bridge_node')
        self.loop = loop
        self.out_queue: asyncio.Queue = asyncio.Queue(loop=loop)
        self.ws_clients = set()

        # Publishers (example)
        # CHECK QOS PROTOCOL (we might want to adjust accordingly)
        self.drive_pub = self.create_publisher(String, '/mc/drive/cmd', 10)

        # Subscribers (example)
        self.create_subscription(String, '/mc/drive/state', self._drive_state_cb, 10)

        self.get_logger().info('MCBridgeNode initialized')

    # TODO: CREATE CALLBACKS FOR EACH TOPIC
    def _drive_state_cb(self, msg: String) -> None:
        payload = {
            'type': 'driveStateReport',
            'data': msg.data,
        }
        # schedule the payload to be sent to websocket clients
        asyncio.run_coroutine_threadsafe(self.out_queue.put(payload), self.loop)

    def handle_ws_message(self, message: Dict[str, Any]) -> None:
        """Handle an incoming JSON message from mission control.

        This is executed in the ROS thread; keep it short and use ROS publishers
        to forward intent into the ROS graph.
        """
        mtype = message.get('type')
        if mtype == 'driveRequest':
            msg = String()
            msg.data = json.dumps(message)
            self.drive_pub.publish(msg)
            self.get_logger().info('Published driveRequest to /mc/drive/cmd')
        elif mtype == 'cameraStreamOpenRequest':
            self.get_logger().info(f"Camera open request: {message}")
        elif mtype == 'cameraStreamCloseRequest':
            self.get_logger().info(f"Camera close request: {message}")
        else:
            self.get_logger().warn(f'Unhandled WS message type: {mtype}')


async def consumer_handler(websocket, node: MCBridgeNode):
    async for message in websocket:
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            node.get_logger().error('Invalid JSON received from WS client')
            continue
        # Forward into ROS thread
        node.get_logger().info(f"WS -> ROS: {data.get('type')}")
        node.handle_ws_message(data)


async def producer_handler(websocket, node: MCBridgeNode):
    while True:
        payload = await node.out_queue.get()
        try:
            await websocket.send(json.dumps(payload))
        except Exception as e:
            node.get_logger().error(f'Error sending to WS client: {e}')
            break


async def ws_handler(websocket, path, node: MCBridgeNode):
    # Accept only the expected mission-control path
    if path != '/mission-control':
        await websocket.close(code=4000, reason='Invalid path')
        return

    node.get_logger().info('Mission control connected')
    node.ws_clients.add(websocket)
    try:
        consumer = asyncio.create_task(consumer_handler(websocket, node))
        producer = asyncio.create_task(producer_handler(websocket, node))
        done, pending = await asyncio.wait([consumer, producer], return_when=asyncio.FIRST_EXCEPTION)
        for p in pending:
            p.cancel()
    finally:
        node.ws_clients.discard(websocket)
        node.get_logger().info('Mission control disconnected')


def start_rclpy_spin(node: MCBridgeNode) -> None:
    try:
        rclpy.spin(node)
    except Exception:
        pass


def _make_process_request():
    # return a small function that rejects non-/mission-control paths early
    async def process_request(path, request_headers):
        if path != '/mission-control':
            return 404, [('Content-Type', 'text/plain')], b'Not Found'
        return None

    return process_request


def main() -> None:
    if websockets is None:
        print('Missing dependency: websockets. Install with `pip install websockets`')
        return

    logging.basicConfig(level=logging.INFO)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    rclpy.init()
    node = MCBridgeNode(loop)

    # Spin ROS in a background thread so asyncio can run in main thread
    spin_thread = threading.Thread(target=start_rclpy_spin, args=(node,), daemon=True)
    spin_thread.start()

    # Start websocket server
    server_coro = websockets.serve(lambda ws, path: ws_handler(ws, path, node), '0.0.0.0', 3001, process_request=_make_process_request())
    server = loop.run_until_complete(server_coro)
    node.get_logger().info('WebSocket server listening on ws://0.0.0.0:3001/mission-control')

    # graceful shutdown handlers
    def _shutdown(signum, frame):
        node.get_logger().info('Shutting down...')
        server.close()
        loop.call_soon_threadsafe(loop.stop)
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        loop.run_forever()
    finally:
        server.close()
        loop.run_until_complete(server.wait_closed())
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
