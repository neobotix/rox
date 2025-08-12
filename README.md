# **rox**

| ![ROX GIF](https://github.com/user-attachments/assets/bf82ae8e-1110-402b-94e7-8f044994d47d) | ![EMROX GIF](https://github.com/user-attachments/assets/25745945-d705-4afa-8b5d-e8ab2f7196a4) |
|:--:|:--:|
| **ROX – Omnidirectional Platform** | **EMROX – Omnidirectional Mobile Manipulator** |


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

### **Key Configuration Parameters**
```xml
<!-- World Configuration -->
<arg name="world_name_global" default="$(find rox_description)/worlds/neo_workshop.world"/>

<!-- Gazebo Settings -->
<arg name="gui" default="true"/>
<arg name="use_sim_time" default="true"/>
<arg name="paused" default="false"/>

<!-- Robot Control -->
<arg name="software_joystick" value="true"/>
```

## 🎮 **Simulation Features**

### **Available Plugins**
- **Planar Movement:** `libgazebo_ros_planar_move.so`
- **Laser Scanner:** `libgazebo_ros_laser.so`
- **Camera:** `libgazebo_ros_camera.so`

### **Robot Capabilities**
- 2D planar movement (x, y, yaw)
- Laser scanner simulation
- Camera simulation
- Realistic physics
- Odometry publishing
