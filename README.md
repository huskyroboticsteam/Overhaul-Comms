# Rover communications bridge

This workspace contains the laptop-side adapter between Mission Control and
ROS 2. Mission Control connects to
`ws://localhost:3001/mission-control`; ROS 2 traffic remains transport-neutral.

## Development

Open the repository in its development container, then build and run:

```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
ros2 run mc_bridge mc_bridge
```

The container uses ROS 2 Jazzy, Cyclone DDS, and the pinned Python dependencies
in `.devcontainer/requirements.txt`. Cyclone DDS is restricted to loopback by
`config/cyclone/laptop.xml`, ready for a colocated ROS2DDS-to-Zenoh bridge.
The container is currently the authoritative runtime; Noble's
`python3-websockets` package is older than the API used by this bridge.

## Current packet routing

- A valid `driveRequest` is published as compact JSON on `mc/drive/cmd`.
- A report packet with a valid envelope received on `mc/report` is sent to the
  active WebSocket controller.
- Camera open and close requests are accepted and logged but not yet routed.
- Unknown packet types are logged and ignored.

The JSON-in-`std_msgs/String` topics are a temporary compatibility boundary.
They will be replaced with shared typed ROS interfaces as the packet contract is
implemented.

WebSocket host, port, path, queue capacities, topics, and QoS depth are ROS
parameters. The server binds to loopback by default and accepts one controller.

## Continuous integration

GitHub Actions runs on pull requests, merge queues, and pushes to `main`. It
checks Python linting, types, tests, and configuration files; builds and tests
the workspace on ROS 2 Jazzy; and builds and smoke-tests the development
container. These three jobs are intended to be required branch-protection
checks.
