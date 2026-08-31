# Rover communications bridge

This ROS 2 Jazzy workspace contains the transport-neutral communication path
between Mission Control and rover nodes. Mission Control remains unchanged and
connects to `ws://localhost:3001/mission-control`.

```text
Mission Control -> WebSocket -> mc_bridge -> typed ROS topics
    -> Zenoh ROS2DDS transport (next step) -> rover_safety -> safe wheel targets
```

## Packages

- `mc_bridge`: validates the versioned packet contract and translates drive,
  tank-drive, and emergency-stop requests to typed ROS messages.
- `rover_interfaces`: shared command and safety message definitions.
- `rover_safety`: rover-local heartbeat watchdog and latched emergency-stop
  gate. This node is independent of Mission Control, the laptop, and Zenoh.

The canonical WebSocket contract is
[`protocol/packet.schema.json`](protocol/packet.schema.json). Its companion
[`protocol/README.md`](protocol/README.md) records compatibility status and
known drift from Mission Control and Resurgence.

## Development

Open the repository in its development container, then build and test:

```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For a single-machine sanity check, run the bridge and watchdog in separate
terminals after sourcing the workspace:

```bash
ros2 run mc_bridge mc_bridge
ros2 run rover_safety rover_watchdog
```

The bridge publishes a connection-scoped heartbeat while its WebSocket client
is healthy. Every command carries the same random session ID plus a monotonic
sequence. It refreshes the active motion and e-stop state only within that
session, clears them on disconnect, and publishes neutral drive immediately.
The rover gate also stops locally after 300 ms without a refreshed motion
command or 500 ms without a heartbeat, rejects stale or reordered motion,
latches every emergency stop, and never resumes old motion after a reconnect or
e-stop clear.

The safe normalized output is `drive/safe_wheels`. Arcade drive matches the
legacy steering convention: `left = straight + steer` and
`right = straight - steer`, clamped to `[-1, 1]`. A hardware-specific drive
node should be the only consumer that converts this output into motor units.

WebSocket host, port, path, queue capacities, topics, and QoS depth are ROS
parameters. The server binds to loopback by default and accepts one controller.

The container selects Cyclone DDS and restricts discovery to loopback using
`config/cyclone/laptop.xml`. Zenoh configuration and Compose orchestration are
deliberately the next end-to-end step, after this local vertical slice.

## Continuous integration

GitHub Actions runs on pull requests, merge queues, and pushes to `main`. It
checks Python linting, types, tests, and configuration files; builds and tests
the workspace on ROS 2 Jazzy; and builds and smoke-tests the development
container. These three jobs are intended to be required branch-protection
checks.

## Platform check

The software baseline is ROS 2 Jazzy on Ubuntu 24.04. The exact rover hardware
and JetPack release still require one read-only check on the physical Jetson;
see [`docs/platform.md`](docs/platform.md).
