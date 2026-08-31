FROM ros:jazzy-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

ENV ROS_WORKSPACE=/home/${USERNAME}/ros2_ws \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///home/${USERNAME}/ros2_ws/config/cyclone/laptop.xml

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-jazzy-rmw-cyclonedds-cpp \
        sudo \
    && rm -rf /var/lib/apt/lists/*

COPY ./requirements.txt /tmp/requirements.txt
RUN python3 -m pip install \
        --break-system-packages \
        --no-cache-dir \
        --requirement /tmp/requirements.txt \
    && rm /tmp/requirements.txt

RUN existing_container_group="$(getent group "$GID" | cut -d: -f1)" \
    && if [ -z "$existing_container_group" ]; then \
        groupadd --gid "$GID" "$USERNAME"; \
    elif [ "$existing_container_group" != "$USERNAME" ]; then \
        groupmod --new-name "$USERNAME" "$existing_container_group"; \
    fi \
    && existing_container_user="$(getent passwd "$UID" | cut -d: -f1)" \
    && if [ -z "$existing_container_user" ]; then \
        useradd --create-home --uid "$UID" --gid "$USERNAME" \
            --shell /bin/bash "$USERNAME"; \
    elif [ "$existing_container_user" != "$USERNAME" ]; then \
        usermod --login "$USERNAME" --home "/home/$USERNAME" \
            --move-home "$existing_container_user"; \
    fi \
    && usermod --gid "$USERNAME" "$USERNAME" \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

RUN mkdir -p "${ROS_WORKSPACE}/src" \
    && printf '%s\n' \
        'source "/opt/ros/${ROS_DISTRO}/setup.bash"' \
        '[ ! -f "${ROS_WORKSPACE}/install/setup.bash" ] || source "${ROS_WORKSPACE}/install/setup.bash"' \
        >> /home/$USERNAME/.bashrc
WORKDIR ${ROS_WORKSPACE}

COPY --chmod=0755 ./ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
