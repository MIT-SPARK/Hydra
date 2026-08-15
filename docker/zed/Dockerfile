# Base image with ZED SDK 5.0, CUDA 12.8, and Ubuntu 24.04
# (while the image name says 12.6, the image appears to have 12.8)
FROM stereolabs/zed:5.0-gl-devel-cuda12.6-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

#setup ros2 workspace (mounted from host)
WORKDIR /root/hydra_ws

#ros2 jazzy
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/trusted.gpg.d/ros.gpg && \
    echo "deb http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-desktop python3-rosdep python3-colcon-common-extensions && \
    apt-get install -y ros-${ROS_DISTRO}-depth-image-proc ros-${ROS_DISTRO}-rqt-tf-tree && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

#install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    usbutils \
    git \
    curl \
    wget \
    tmux \
    vim \
    htop \
    net-tools \
    ros-dev-tools \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libcli11-dev \
    libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

#install pipx + rosbags
RUN apt-get update && apt-get install -y pipx && \
    pipx install rosbags && \
    echo 'export PATH=$PATH:/root/.local/bin' >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/*

#install tensorrt (must remove conflicting cuda repo to make work, unsure why
#cuda12.8 comes with the base image instead of cuda12.6)
RUN rm -f /etc/apt/sources.list.d/cuda-ubuntu2404-x86_64.list && \
    rm -f /etc/apt/sources.list.d/cuda*.list && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y \
        libnvinfer-dev=10.8.0.43-1+cuda12.8 \
        libnvinfer10=10.8.0.43-1+cuda12.8 \
        libnvinfer-headers-dev=10.8.0.43-1+cuda12.8 \
        libnvinfer-plugin-dev=10.8.0.43-1+cuda12.8 \
        libnvinfer-plugin10=10.8.0.43-1+cuda12.8 \
        libnvinfer-headers-plugin-dev=10.8.0.43-1+cuda12.8 \
        libnvonnxparsers-dev=10.8.0.43-1+cuda12.8 \
        libnvonnxparsers10=10.8.0.43-1+cuda12.8 && \
    rm -rf /var/lib/apt/lists/* && \
    rm cuda-keyring_1.1-1_all.deb

#copy source code for rosdep (replaced by a volume mount in container)
COPY ../../../src/ /tmp/deps/src/

#install dependencies via rosdep
RUN apt-get update && apt-get install -y libzmq5-dev && \
    rosdep install --from-paths /tmp/deps/src --ignore-src -r -y --rosdistro=${ROS_DISTRO} || true && \
    rm -rf /tmp/deps

#networking
RUN apt update && apt install -y ros-jazzy-rmw-zenoh-cpp

#useful shell configuration
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/hydra_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias cb='cd /root/hydra_ws && colcon build --symlink-install --continue-on-error'" >> /root/.bashrc
