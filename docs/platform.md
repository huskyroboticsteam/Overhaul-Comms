# Jetson platform check

Run these commands on the rover and save their output with the deployment
notes before selecting an arm64 image or GPU-dependent base image:

```bash
tr -d '\0' </proc/device-tree/model
dpkg-query -W nvidia-jetpack 2>/dev/null || true
head -n 1 /etc/nv_tegra_release
. /etc/os-release && printf '%s %s\n' "$NAME" "$VERSION_ID"
uname -m
```

The expected family is Jetson Orin and the required architecture is `aarch64`.
The exact model, JetPack/Jetson Linux release, and host Ubuntu version must be
recorded before pinning the production image. The bridge and watchdog do not
use CUDA, so this fact does not block their ROS-level tests.
