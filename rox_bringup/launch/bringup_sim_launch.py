# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
import os
from pathlib import Path
import xacro

def execution_stage(context: LaunchContext, frame_type, rox_type, arm_type, use_d435, use_imu):
    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', 'neo_workshop.sdf')
    bridge_config_file = os.path.join(get_package_share_directory('rox_bringup'), 'configs/gz_bridge', 'gz_bridge_config.yaml')
    frame_typ = str(frame_type.perform(context))
    arm_typ = str(arm_type.perform(context))
    rox_typ = str(rox_type.perform(context))
    d435 = str(use_d435.perform(context))
    imu = str(use_imu.perform(context))

    if (rox_typ == "meca"):
        frame_typ = "long"
        print("Meca only supports long frame")   
    
    urdf = os.path.join(get_package_share_directory('rox_description'), 'urdf', 'rox.urdf.xacro')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        output='screen',
        arguments=[
            '-topic', "robot_description",
            '-name', "rox"])
    
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        )
        , launch_arguments={'ign_args': ['-r ', default_world_path]}.items()
      )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
            "xacro", " ", urdf, " ", 'frame_type:=',
            frame_typ,
            " ", 'arm_type:=',
            arm_typ,
            " ", 'rox_type:=',
            rox_type,
            " ", 'use_imu:=',
            imu,
            " ", 'use_d435:=',
            d435,
            " ", 'use_gz:=',
            "True"
            ])}],
        arguments=[urdf])
    
    teleop =  Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop')
  
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_file}])

    return [start_robot_state_publisher_cmd, spawn_robot, ignition, gz_bridge, teleop]

def generate_launch_description():
    opq_function = OpaqueFunction(function=execution_stage,
                                  args=[LaunchConfiguration('frame_type'),
                                        LaunchConfiguration('rox_type'),
                                        LaunchConfiguration('arm_type'),
                                        LaunchConfiguration('d435_enable'),
                                        LaunchConfiguration('imu_enable')
                                        ])
    
    declare_frame_type_cmd = DeclareLaunchArgument(
            'frame_type', default_value='short',
            description='Frame type - Options: short/long'
        )
    
    declare_rox_type_cmd = DeclareLaunchArgument(
            'rox_type', default_value='argo',
            description='Robot type - Options: argo/diff/trike/meca'
        )

    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )
    
    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Realsense - Options: True/False'
        )
    
    declare_arm_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            description='Arm used in the robot - currently only support universal'
        )

    return LaunchDescription([
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_arm_cmd,
        declare_frame_type_cmd,
        declare_rox_type_cmd,
        opq_function
    ])
