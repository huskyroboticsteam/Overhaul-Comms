"""Application lifecycle for the Mission Control bridge."""

from __future__ import annotations

import asyncio
import logging
import signal
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from mc_bridge.bridge_node import MCBridgeNode, create_node
from mc_bridge.websocket_server import SingleControllerWebSocketServer


_LOGGER = logging.getLogger(__name__)


async def run_bridge(node: MCBridgeNode) -> None:
    """Run ROS and WebSocket processing until a signal or ROS failure."""
    loop = asyncio.get_running_loop()
    stop_requested = asyncio.Event()
    spin_failure: asyncio.Future[None] = loop.create_future()
    installed_signals: list[signal.Signals] = []

    async def wait_for_stop_request() -> None:
        await stop_requested.wait()

    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(signum, stop_requested.set)
        except (NotImplementedError, RuntimeError):
            continue
        installed_signals.append(signum)

    settings = node.settings
    websocket_server = SingleControllerWebSocketServer(
        node.enqueue_ws_message,
        host=settings.websocket_host,
        port=settings.websocket_port,
        path=settings.websocket_path,
        outbound_capacity=settings.outbound_capacity,
        connect_handler=node.begin_ws_session,
        disconnect_handler=node.end_ws_session,
    )
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)

    def report_spin_failure(error: BaseException) -> None:
        if not spin_failure.done():
            spin_failure.set_exception(error)

    def spin_ros() -> None:
        try:
            executor.spin()
        except BaseException as error:
            loop.call_soon_threadsafe(report_spin_failure, error)
        else:
            loop.call_soon_threadsafe(
                report_spin_failure,
                RuntimeError('ROS executor stopped unexpectedly'),
            )

    spin_thread = threading.Thread(
        target=spin_ros,
        name='mc-bridge-ros-executor',
        daemon=False,
    )

    try:
        async with websocket_server:
            node.set_outbound_publisher(websocket_server.publish_from_thread)
            spin_thread.start()
            _LOGGER.info(
                'WebSocket server listening on %s:%d%s',
                settings.websocket_host,
                websocket_server.bound_port,
                settings.websocket_path,
            )

            stop_task = asyncio.create_task(
                wait_for_stop_request(),
                name='shutdown-signal',
            )
            done, _ = await asyncio.wait(
                {stop_task, spin_failure},
                return_when=asyncio.FIRST_COMPLETED,
            )
            if spin_failure in done:
                spin_failure.result()
            stop_task.cancel()
            await asyncio.gather(stop_task, return_exceptions=True)
    finally:
        node.set_outbound_publisher(None)
        spin_failure.cancel()
        executor.shutdown(timeout_sec=5.0)
        if spin_thread.is_alive():
            spin_thread.join(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        for signum in installed_signals:
            loop.remove_signal_handler(signum)

        if spin_thread.is_alive():
            raise RuntimeError('ROS executor did not stop within five seconds')


def main(args: list[str] | None = None) -> None:
    """Initialize ROS, run the bridge, and shut down cleanly."""
    logging.basicConfig(level=logging.INFO)
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    try:
        asyncio.run(run_bridge(create_node()))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
