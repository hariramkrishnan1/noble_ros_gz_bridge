#!/bin/bash

# Auto-detect ROS_DISTRO
if [ -z "$ROS_DISTRO" ]; then
    echo "[INFO] ROS_DISTRO not set. Trying to auto-detect..."
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        export ROS_DISTRO=jazzy
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        export ROS_DISTRO=humble
    else
        echo "[ERROR] Unable to detect ROS distribution. Please source your ROS environment or set ROS_DISTRO manually."
        exit 1
    fi
fi

echo "[INFO] Using ROS_DISTRO: $ROS_DISTRO"

# Update system and install dependencies
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-ros-gz-bridge

if [ $? -eq 0 ]; then
    echo "[SUCCESS] Dependencies installed successfully."
else
    echo "[ERROR] Failed to install dependencies."
    exit 1
fi
