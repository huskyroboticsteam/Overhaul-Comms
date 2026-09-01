# syntax=docker/dockerfile:1.7

FROM ros:jazzy-ros-base AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/rover_ws

COPY src/ src/
COPY protocol/ protocol/

RUN apt-get update \
    && if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then \
        rosdep init; \
    fi \
    && rosdep update --rosdistro jazzy \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro jazzy \
        --yes \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --merge-install \
        --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /var/lib/apt/lists/* build log

FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_LOG_DIR=/tmp/ros

RUN --mount=type=bind,source=src,target=/tmp/rover_src,ro \
    apt-get update \
    && apt-get install --yes --no-install-recommends \
        python3-pip \
        ros-jazzy-rmw-cyclonedds-cpp \
    && if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then \
        rosdep init; \
    fi \
    && rosdep update --rosdistro jazzy \
    && rosdep install \
        --from-paths /tmp/rover_src \
        --ignore-src \
        --rosdistro jazzy \
        --dependency-types exec \
        --yes \
    && rm -rf /var/lib/apt/lists/*

COPY .devcontainer/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install \
        --break-system-packages \
        --no-cache-dir \
        --requirement /tmp/requirements.txt \
    && rm /tmp/requirements.txt

COPY --from=builder /opt/rover_ws/install/ /opt/rover_ws/install/
COPY scripts/zenoh_integration_test.py /opt/rover_ws/scripts/zenoh_integration_test.py
COPY --chmod=0755 docker/ros_entrypoint.sh /ros_entrypoint.sh

WORKDIR /opt/rover_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
