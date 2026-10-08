FROM osrf/ros:jazzy-desktop-full

# Ubuntu 24.04 images already have a user "ubuntu" with UID 1000; create it only if missing.
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
RUN sudo usermod --append --groups video,dialout $USERNAME

RUN sudo apt-get update && sudo apt-get install -y \
    git \
    ros-dev-tools \
    python-is-python3 \
    python3-pip \
    vim \
    nano \
    tmux

RUN rosdep update
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Micro XRCE-DDS Agent: bridge between PX4 and ROS 2
RUN git clone --depth 1 -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent && \
    cmake -S /tmp/agent -B /tmp/agent/build && \
    cmake --build /tmp/agent/build -j$(nproc) && \
    sudo cmake --install /tmp/agent/build && \
    sudo ldconfig && \
    rm -rf /tmp/agent

# PX4 SITL, pre-built. px4_msgs must match: release/1.16
RUN git clone --recursive --depth 1 --shallow-submodules -b v1.16.0 \
        https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot && \
    bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx && \
    cd ~/PX4-Autopilot && make px4_sitl

# px4_msgs (ROS 2 PX4) in its own workspace, matching the PX4 version above
RUN mkdir -p ~/px4_ws/src && \
    git clone --depth 1 -b release/1.16 https://github.com/PX4/px4_msgs.git ~/px4_ws/src/px4_msgs && \
    cd ~/px4_ws && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

RUN echo "source ~/px4_ws/install/setup.bash" >> ~/.bashrc
RUN echo "[ -f ~/vespr/install/setup.bash ] && source ~/vespr/install/setup.bash" >> ~/.bashrc

# PX4 DDS domain must match ROS_DOMAIN_ID
ENV ROS_DOMAIN_ID=16
ENV PX4_PARAM_UXRCE_DDS_DOM_ID=16

# Keeping ROS 2 and Gazebo traffic local for testing
# To talk to the drone or mocap: export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV GZ_IP=127.0.0.1

CMD ["bash"]
