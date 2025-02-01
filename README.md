# Dual UR Setup Guide

## Prerequisites
- ROS2 Jazzy
- Ubuntu 24

## Installation Steps

### 1. Create Workspace
```bash
export DUAL_ARM_WS=~/your_ws/
mkdir -p $DUAL_ARM_WS
cd $DUAL_ARM_WS
```

### 2. Clone Repository
```bash
git clone --recurse-submodules https://github.com/sl148/dual_ur.git src
```

### 3. Build MoveIt2 from Source

- Install Dependencies
    ```bash
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
    ```

- Update System
    ```bash
    sudo apt update
    sudo apt dist-upgrade -y
    rosdep update
    source /opt/ros/$ROS_DISTRO/setup.bash
    ```

- Uninstall Any Pre-existing MoveIt Debians
    ```bash
    sudo apt remove -y ros-$ROS_DISTRO-moveit*
    ```

- Download Dependencies
    ```bash
    cd $DUAL_ARM_WS/src
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- Set Middleware
    ```bash
    sudo apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
    ```

### 4. Install Gazebo UR Dependencies
```bash
rosdep update && rosdep install --ignore-src --from-paths src -y
```

### 4. Source Robotiq Dependencies
```bash
# There might be a workaround, but this works
git clone https://github.com/tylerjw/serial.git -b ros2 $DUAL_ARM_WS/src/ros2_robotiq_gripper/
```

### 5. Build Workspace
```bash
cd $DUAL_ARM_WS
colcon build --symlink-install
```

### 6. Running the Dual UR System
Once the setup is complete, you can launch the system using:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch dual_ur_robotiq_rs_moveit_config dual_ur_robotiq_rs.launch
```

