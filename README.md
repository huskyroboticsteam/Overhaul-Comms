# Rover communications bridge

This ROS 2 Jazzy workspace contains the transport-neutral communication path
between Mission Control and rover nodes. Mission Control remains unchanged and
connects to `ws://localhost:3001/mission-control`.

```text
Mission Control
  <-> ws://localhost:3001/mission-control
mc_bridge (laptop)
  <-> laptop-local Cyclone DDS
zenoh-bridge-ros2dds (laptop, client)
  <-> Zenoh TCP
zenoh-bridge-ros2dds (rover, router)
  <-> rover-local Cyclone DDS
rover_safety and rover ROS nodes
```

## Packages

- `mc_bridge`: validates and translates every v1 request and report using typed
  ROS interfaces.
- `rover_interfaces`: shared command, telemetry, camera, and safety messages.
- `rover_safety`: rover-local heartbeat watchdog and latched emergency-stop
  gate for wheel and continuous joint-power outputs.

The canonical WebSocket contract is
[`protocol/packet.schema.json`](protocol/packet.schema.json). Its companion
[`protocol/README.md`](protocol/README.md) records compatibility status and
known drift from Mission Control and Resurgence.

## Compose deployment

Start the rover first:

```bash
docker compose --profile rover up --detach --build
```

On the Mission Control laptop, point Zenoh at the rover radio address:

```bash
ROVER_ZENOH_ENDPOINT=tcp/192.168.1.10:7447 \
  docker compose --profile laptop up --detach --build
```

Successful `main` and manually dispatched CI runs publish the tested
multi-platform image to GitHub Container Registry. To deploy it without a
local build:

```bash
export OVERHAUL_COMMS_IMAGE=ghcr.io/huskyroboticsteam/overhaul-comms:main
docker compose --profile rover pull
docker compose --profile rover up --detach --no-build
```

Private packages require `docker login ghcr.io` first. Each CI run also offers
seven-day, directly downloadable files named
`overhaul-comms-amd64-<commit>.tar` and
`overhaul-comms-arm64-<commit>.tar`. Loading the appropriate file installs the
default `overhaul-comms:local` image used by Compose:

```bash
docker load --input overhaul-comms-arm64-<commit>.tar
docker compose --profile rover up --detach --no-build
```

The `laptop` and `rover` profiles may run on different amd64 or arm64 systems.
Both use `zenoh-bridge-ros2dds:1.10.0`, disable discovery across the radio, and
allow only the named command and report topics in `config/zenoh/`.

For a complete single-host qualification run:

```bash
docker compose --profile integration up --build \
  --abort-on-container-exit \
  --exit-code-from integration_test \
  integration_test
docker compose --profile integration down --volumes
```

Cyclone DDS is intentionally restricted to each container network namespace.
Additional rover nodes must share `rover_zenoh`'s network namespace, as
`rover_safety` does in `compose.yaml`. Host-native ROS nodes require a separate
host-network deployment rather than these loopback-only container settings.

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

The bridge validates input, accepts one controller, and gives commands a random
session ID plus monotonic sequence. Drive and joint-power controls are bounded
latest values, refreshed only while connected, and neutralized on disconnect,
mode change, or e-stop. The rover watchdog independently rejects stale sessions
and stops after a command or heartbeat timeout. A heartbeat timeout retires the
session, so Mission Control must reconnect before the rover accepts commands.
The session-start lease is volatile, which enforces the same reconnect after a
rover watchdog restart instead of replaying the laptop's old intent. Compose
also persists the rover-local emergency-stop latch until an explicit healthy
clear is accepted.

Safe outputs are `drive/safe_wheels`, `arm/safe_joint_power`,
`camera/safe_capture`, and `camera/safe_stream`. Hardware nodes should consume
only these gated outputs. Camera captures are volatile events; stream state is
retained as one atomic snapshot containing every camera. The camera lease
publishes stream closes after heartbeat loss, e-stop, disconnect, or watchdog
restart. One-shot camera commands and e-stop clears are held briefly if Zenoh
delivers them ahead of their matching heartbeat; stop assertions remain
immediate.
Arcade drive preserves the legacy convention:
`left = straight + steer` and `right = straight - steer`, clamped to
`[-1, 1]`.

Rover consumers for position, IK, servo, stepper, and waypoint commands must
also treat `safety/state` as a lease and cancel their hardware-specific action
when it becomes unsafe or stops refreshing. A safe lease has a nonzero matching
session and neither `heartbeat_timed_out` nor `emergency_stop_latched` set;
`command_timed_out` applies to wheel motion. Those actuator/navigation
consumers are outside this communications workspace, where a generic neutral
target would be ambiguous.

Camera conversion runs in a bounded latest-frame worker. JPEG frames become
base64 packets; H.264 frames preserve their Annex-B NAL boundaries. Responses
carry internal session/request identity so late frames are discarded after a
close, replacement request, or reconnect.

WebSocket settings, queue limits, topic names, rates, payload limits, and QoS
depths are ROS parameters. The application packages contain no Zenoh APIs.

## Continuous integration

GitHub Actions checks linting, strict types, packet fixtures, ROS Jazzy builds,
the development container, amd64/arm64 production images, and the production
Compose path. The integration job creates two isolated DDS graphs and verifies
drive, tank drive, disconnect/reconnect, e-stop latch/clear, return telemetry,
and topic-allowlist isolation through Zenoh, plus the rover-local camera lease.
Production images are retained as named tar artifacts and published to GHCR
only after every required job passes. Docker's diagnostic `.dockerbuild`
uploads are disabled.

## Platform check

The software baseline is ROS 2 Jazzy on Ubuntu 24.04. The exact rover hardware
and JetPack release still require one read-only check on the physical Jetson;
see [`docs/platform.md`](docs/platform.md).
