# **rox**

| ![ROX GIF](https://github.com/user-attachments/assets/bf82ae8e-1110-402b-94e7-8f044994d47d) | ![EMROX GIF](https://github.com/user-attachments/assets/25745945-d705-4afa-8b5d-e8ab2f7196a4) |
|:--:|:--:|
| **ROX – Omnidirectional Platform** | **EMROX – Omnidirectional Mobile Manipulator** |

# ROS Noetic Real robot Setup Guide

## **Getting Started with the Real Robot**

Please follow the instructions given Starting with [ROS 2 on the Robot from our documentation website](https://neobotix-docs.de/ros/ros2/starting_with_ROS.html). The equivalent autostart scripts and desktop applications are in place to start the robot. Similar to ROS 2, you can also use commandline for starting ROS

```bash
roslaunch rox_bringup bringup.launch
```

If you would like to reconfigure the autostart bringup or the desktop application, then please check the ros_settings.sh script and ROS_AUTOSTART.sh script located in `/home/neobotix/`

If you are looking to visualize the robot in RViz, you can simply use:

```bash
roslaunch rox_rviz rox_rviz.launch
```

Mapping using Gmapping can be started using the following command

```bash
#Start Mapping
roslaunch rox_navigation gmapping_basic.launch

#start rviz to see the map
roslaunch rox_rviz rviz_navigation.launch
```

Autonomous navigation utilizing the move_base has also been configured, simply use the following commands to launch the autonomous navigation:

```bash
#start navigation
roslaunch rox_navigation navigation_basic_neo.launch

#start rviz to send goals
roslaunch rox_rviz rviz_navigation.launch
```

By default neo_local_planner along with NavFn planner has been configured for autonomous navigation. Feel free to change and update the configs according to your requirements at:

`/home/neobotix/ros_workspace/src/rox/rox_navigation/configs`


# ROS Noetic Simulation Setup Guide

## 🚀 **Installation Steps**

### **1. System Requirements**
- Ubuntu 20.04 LTS (Focal Fossa)
- ROS Noetic (Desktop-Full recommended)
- At least 4GB RAM
- 10GB free disk space

### **2. Install ROS Noetic (if not already installed)**

### **3. Install Gazebo and ROS Gazebo Packages**
```bash
# Update package list
sudo apt update

# Install Gazebo 11 and ROS Gazebo packages
sudo apt install -y gazebo11 libgazebo11-dev \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-dev \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-msgs
```

### **4. Install Additional ROS Packages**

```bash
# Install required ROS packages for simulation
sudo apt install -y \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-xacro \
    
```

### **5. Install Navigation Packages**

```bash
# Install navigation packages for autonomous movement
sudo apt install -y \
    ros-noetic-amcl \
    ros-noetic-neo-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base

# Clone additional navigation packages from GitHub
cd /home/ros_workspace/src

# Clone neo_localization for robot localization
git clone https://github.com/neobotix/neo_localization.git

# Clone gmapping for SLAM (Simultaneous Localization and Mapping)
git clone https://github.com/neobotix/slam_gmapping.git
```

### **6. Set Environment Variables**
```bash
# Set Gazebo model path to your workspace
echo 'export GAZEBO_MODEL_PATH=/your_workspace/src/rox/rox_description/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc

# Source the updated bashrc
source ~/.bashrc
```

### **7. Build Your Workspace**
```bash
# Navigate to your workspace
cd /home/ros_workspace

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash

# Note: After installing navigation packages, rebuild the workspace
# catkin_make
# source devel/setup.bash
```

## 🎯 **Quick Start Simulation**

### **1. Start the Simulation**
```bash
# Terminal 1: Launch the simulation
roslaunch rox_bringup bringup_sim.launch
```

### **2. Control the Robot**
```bash
# Terminal 2: Use keyboard teleop (if enabled)
# The teleop node is already included in the launch file
# Use these keys to control the robot:
#   i: Forward
#   ,: Backward
#   j: Turn left
#   l: Turn right
#   k: Stop
#   Shift+J: Strafe left
#   Shift+L: Strafe right
```

### **3. View Robot State**
```bash
# Terminal 3: Check robot state
rostopic echo /robot_state_publisher

# Check odometry
rostopic echo /odom

# Check velocity commands
rostopic echo /cmd_vel
```

### **4. Launch Navigation (Optional)**
```bash
# Terminal 5: Launch navigation for autonomous movement
roslaunch rox_navigation navigation_basic_neo.launch
```

### **5. Launch RViz (Optional)**
```bash
# Terminal 4: Launch RViz for navigation visualization
roslaunch rox_rviz rviz_navigation.launch
```

## 🔧 **Configuration Files**

### **Launch File Location**
- **Main Simulation:** `src/rox/rox_bringup/launch/bringup_sim.launch`
- **Robot Description:** `src/rox/rox_description/urdf/rox.urdf.xacro`
- **Gazebo Configuration:** `src/rox/rox_description/urdf/xacros/gazebo.xacro`

