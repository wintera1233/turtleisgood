FROM osrf/ros:noetic-desktop-full

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y \
    sudo \
    git \
    wget \
    vim \
    bash-completion \
    build-essential \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -m -s /bin/bash -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN mkdir -p /home/${USERNAME}/ros1-ws/src/devel-ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc

ENV ROS_WS=/home/${USERNAME}/ros1-ws
WORKDIR ${ROS_WS}