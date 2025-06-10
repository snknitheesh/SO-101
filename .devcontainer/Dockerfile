FROM rwthika/ros2-ml:latest AS builder-base

ENV ROS_DISTRO=humble
ARG CR_PAT
ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ARG ISAAC_VERSION

# Ros requirements
COPY /ros/additional_ros_requirements.txt /additional_ros_requirements.txt
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    xargs apt-get install -y < /additional_ros_requirements.txt

# Pip requirements
COPY ros/additional_pip_requirements.txt tmp/additional_pip_requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install -r tmp/additional_pip_requirements.txt

# Deb requirements
COPY ros/additional_deb_requirements.txt /additional_deb_requirements.txt
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    xargs apt-get install -y < /additional_deb_requirements.txt && \
    rm -rf /var/lib/apt/lists/*
