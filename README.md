# Overhaul-Comms

Mission control ↔ rover communication.

```
Web app ──WS──▶ bridge_node ─DDS─▶ zenoh-bridge ══ Zenoh (TCP 7447) ══▶ zenoh-bridge ─DDS─▶ rover nodes
                └──────────── laptop ────────────┘                      └──────────── rover ─────────────┘
```

DDS stays on each machine (`ROS_LOCALHOST_ONLY=1`); only Zenoh crosses the network.

## Requirements

### Laptop (mission control)
- Docker + VS Code Dev Containers. The devcontainer installs ROS 2 Humble, CycloneDDS and `zenoh-bridge-ros2dds`.
- macOS: enable host networking in Docker Desktop (Settings → Resources → Network).

### Rover (Ubuntu 22.04, ROS 2 Humble)
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
curl -fsSL https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes -o /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
# 1.9.0 matches the laptop; 1.10+ doesn't install on Ubuntu 22.04 (amd64)
sudo apt update && sudo apt install zenoh-bridge-ros2dds=1.9.0
sudo apt-mark hold zenoh-bridge-ros2dds

# Start the bridge on boot (the package ships a disabled service; this adds our settings to it)
sudo mkdir -p /etc/systemd/system/zenoh-bridge-ros2dds.service.d
sudo cp deploy/rover/zenoh-bridge-ros2dds.service.d/override.conf /etc/systemd/system/zenoh-bridge-ros2dds.service.d/
sudo systemctl daemon-reload
sudo systemctl enable --now zenoh-bridge-ros2dds
journalctl -u zenoh-bridge-ros2dds -f   # logs
```

Every ROS node on the rover must run with:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
```

## Running (laptop)

In the devcontainer:
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch mc_bridge_pkg mission_control.launch.py rover_ip:=<rover-ip>
```

Or with the image CI publishes:
```bash
docker run --rm --net=host --cap-add=NET_ADMIN ghcr.io/huskyroboticsteam/overhaul-comms:latest \
  ros2 launch mc_bridge_pkg mission_control.launch.py rover_ip:=<rover-ip>
```

## Tests
```bash
colcon test && colcon test-result --verbose
```

CI (`.github/workflows/ci.yml`) runs the tests and pushes an image to GHCR on every push to `main` and every PR commit (`latest` and `main` tags from `main`, `pr-<n>` from PRs, plus `sha-<commit>`).
