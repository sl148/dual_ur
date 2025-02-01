#!/bin/bash

# Dual UR Setup Script

# 1. Create Workspace
export DUAL_ARM_WS=~/your_ws/
mkdir -p $DUAL_ARM_WS
cd $DUAL_ARM_WS

# 2. Clone Repository
git clone --recurse-submodules https://github.com/sl148/dual_ur.git src

# 3. Build MoveIt2 from Source

# Install Dependencies
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget

# Update System
sudo apt update
sudo apt dist-upgrade -y
rosdep update
source /opt/ros/$ROS_DISTRO/setup.bash

# Uninstall Any Pre-existing MoveIt Debians
sudo apt remove -y ros-$ROS_DISTRO-moveit*

# Download Dependencies
cd $DUAL_ARM_WS/src
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Set Middleware
sudo apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# 4. Install Gazebo UR Dependencies
rosdep update && rosdep install --ignore-src --from-paths src -y

# 5. Build Workspace
cd $DUAL_ARM_WS
colcon build --symlink-install

echo "Dual UR setup is complete!"


# How to run
ros2 launch dual_ur_robotiq_rs_moveit_config dual_ur_robotiq_rs.launch
