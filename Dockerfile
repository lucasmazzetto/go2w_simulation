FROM ros:jazzy-ros-base AS dependencies

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Mesa supports software rendering when no GPU is passed into the container.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libgl1-mesa-dri \
        python3-colcon-common-extensions \
        ros-jazzy-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/go2w_ws

# Fetch only official ROS interfaces; the robot SDK and custom DDS build are unnecessary.
COPY dependencies.repos /tmp/dependencies.repos
COPY docker/import_unitree.py /tmp/import_unitree.py
RUN python3 /tmp/import_unitree.py

# Cache dependency installation separately from the model and launch files.
COPY go2w_description/package.xml src/go2w_description/package.xml
COPY go2w_gazebo/package.xml src/go2w_gazebo/package.xml
RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
    && rm -rf /var/lib/apt/lists/*

FROM dependencies AS runtime
COPY go2w_description/ src/go2w_description/
COPY go2w_gazebo/ src/go2w_gazebo/
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --merge-install --packages-up-to go2w_gazebo --event-handlers console_direct+ \
    && chmod -R a+rwX build \
    && rm -rf log

COPY docker/unitree.LICENSE /opt/go2w_ws/src/unitree_go/LICENSE
COPY docker/bashrc /opt/go2w_ws/docker/bashrc

COPY --chmod=755 docker/entrypoint.sh /go2w_entrypoint.sh

ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/go2w_entrypoint.sh"]
CMD ["ros2", "launch", "go2w_gazebo", "sim.launch.py"]
