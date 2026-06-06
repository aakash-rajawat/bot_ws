# syntax=docker/dockerfile:1

ARG CUDA_IMAGE=nvidia/cuda:12.5.1-devel-ubuntu24.04

FROM ${CUDA_IMAGE} AS base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    locales \
    lsb-release \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-venv \
    ros-${ROS_DISTRO}-ros-base \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true \
    && mkdir -p /etc/ros/rosdep/sources.list.d

RUN if id -u ubuntu >/dev/null 2>&1; then \
      usermod --shell /bin/bash ubuntu; \
      mkdir -p /home/ubuntu; \
      chown ubuntu:ubuntu /home/ubuntu; \
    else \
      useradd --uid 1000 --create-home --shell /bin/bash ubuntu; \
    fi

WORKDIR /tmp/bot_ws

RUN mkdir -p /tmp/bot_ws/rosdep

COPY rosdep/base.yaml /tmp/bot_ws/rosdep/base.yaml
COPY src /tmp/bot_ws/src

RUN echo "yaml file:///tmp/bot_ws/rosdep/base.yaml" \
      > /etc/ros/rosdep/sources.list.d/00-bot-ws.list \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

FROM base AS dev

RUN apt-get update && apt-get install -y --no-install-recommends sudo \
    && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && rm -rf /var/lib/apt/lists/*

USER ubuntu
WORKDIR /workspaces/bot_ws

CMD ["/bin/bash"]

FROM base AS runtime

WORKDIR /workspaces/bot_ws

COPY --chown=ubuntu:ubuntu . /workspaces/bot_ws

RUN chown ubuntu:ubuntu /workspaces/bot_ws

USER ubuntu

RUN bash scripts/fetch_third_party.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --merge-install \
    && bash scripts/bootstrap_workspace_venvs.sh /workspaces/bot_ws \
    && rm -rf build log

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /workspaces/bot_ws/install/setup.bash && ros2 launch bot_bringup vision_container_v1.launch.py"]
