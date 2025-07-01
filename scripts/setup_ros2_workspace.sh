#!/bin/bash

# ROS2 Jazzy Installation and Workspace Setup Script for Ubuntu 24.04
# This script sets up a complete ROS2 environment for robot simulation

echo "=== ROS2 Jazzy Installation and Workspace Setup ==="
echo "This script will install ROS2 Jazzy and set up your workspace"
echo "Make sure you're running this on WSL2 Ubuntu 24.04"

# Update system
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install required tools
echo "Installing required tools..."
sudo apt install -y curl gnupg2 lsb-release software-properties-common

# Add ROS2 repository
echo "Adding ROS2 repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
sudo apt update

# Install ROS2 Jazzy Desktop Full
echo "Installing ROS2 Jazzy Desktop Full..."
sudo apt install -y ros-jazzy-desktop-full

# Install additional packages for robot simulation
echo "Installing additional ROS2 packages..."
sudo apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git

# Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init
rosdep update

# Setup environment
echo "Setting up ROS2 environment..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Create workspace
echo "Creating ROS2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Build workspace
echo "Building workspace..."
colcon build

# Setup workspace environment
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "=== Installation Complete ==="
echo "Please run 'source ~/.bashrc' or restart your terminal"
echo "Then you can copy your arm_urdf package to ~/ros2_ws/src/"
