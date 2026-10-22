# Mission control laptop image: bridge_node + Zenoh bridge to the rover.
# The devcontainer (.devcontainer/Dockerfile) is for development; this one is built by CI.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update -q \
    && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    iproute2 \
    python3-pip \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key \
    | gpg --dearmor --yes -o /etc/apt/keyrings/zenoh-public-key.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" \
    > /etc/apt/sources.list.d/zenoh.list \
    && apt-get update -q \
    && apt-get install -y --no-install-recommends zenoh-bridge-ros2dds \
    && rm -rf /var/lib/apt/lists/*

COPY .devcontainer/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

WORKDIR /ros2_ws
COPY src/ src/

# Install package dependencies (including test deps so CI can run colcon test), then build
RUN apt-get update -q \
    && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/"$ROS_DISTRO"/setup.bash \
    && colcon build

# Keep DDS on this machine; the Zenoh bridge carries traffic to the rover
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=1

COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "mc_bridge_pkg", "mission_control.launch.py"]
