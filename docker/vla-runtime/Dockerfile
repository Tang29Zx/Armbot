FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-msgpack \
        python3-numpy \
        python3-pil \
        python3-pip \
        ros-humble-rmw-fastrtps-cpp \
    && python3 -m pip install --no-cache-dir 'websockets>=11,<16' \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/armbot_ws

COPY ros2_ws/src/action_interfaces src/action_interfaces
COPY ros2_ws/src/vla_runtime src/vla_runtime

RUN source /opt/ros/humble/setup.bash \
    && colcon build \
        --merge-install \
        --packages-select action_interfaces vla_runtime

COPY docker/vla-runtime/fastdds.pc.xml.in /etc/armbot/fastdds.pc.xml.in
COPY docker/vla-runtime/entrypoint.sh /usr/local/bin/armbot-vla-entrypoint

ENTRYPOINT ["/usr/local/bin/armbot-vla-entrypoint"]
CMD ["ros2", "launch", "vla_runtime", "vla_bridge.launch.py"]

