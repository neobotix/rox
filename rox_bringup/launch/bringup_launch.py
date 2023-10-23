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

def execution_stage(context: LaunchContext, robot_type):

  # ToDo: Namespacing
  rox_type = robot_type.perform(context)

  # teleop
  teleop = Node(
          package='neo_teleop2',
          executable='neo_teleop2_node',
          output='screen',
          name='neo_teleop2_node',
          parameters = [os.path.join(get_package_share_directory('rox_bringup'),'configs/teleop', rox_type + '_teleop.yaml')])
  # joy
  joy = Node(
          package='joy', 
          executable='joy_node', 
          output='screen',
          name='joy_node',
          parameters = [{'dev': "/dev/input/js0"}, {'deadzone':0.12}])


  # robot_state_publisher
  # ToDo: Use Xacro to select the specific robot type
  urdf = os.path.join(get_package_share_directory('rox_description'), 'urdf', 'rox.urdf')

  with open(urdf, 'r') as infp:  
      robot_desc = infp.read()

  start_robot_state_publisher_cmd = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'robot_description': robot_desc}],
      arguments=[urdf])
  
  # laser scanners
  scan1 = Node(
          package="sick_safetyscanners2",
          executable="sick_safetyscanners2_node",
          name="sick_safetyscanners2_node",
          output="screen",
          emulate_tty=True,
          parameters=[os.path.join(get_package_share_directory('rox_bringup'),'configs/sensors', 'nanoscan_1.yaml')],
          remappings=[
              ('/scan', '/lidar_1/scan_filtered')
          ]
      )

  # laser scanners
  scan2 = Node(
          package="sick_safetyscanners2",
          executable="sick_safetyscanners2_node",
          name="sick_safetyscanners2_node",
          output="screen",
          emulate_tty=True,
          parameters=[os.path.join(get_package_share_directory('rox_bringup'),'configs/sensors', 'nanoscan_2.yaml')],
          remappings=[
              ('/scan', '/lidar_2/scan_filtered'),
          ]
      )

  # relayboard node
  relayboard = Node(
          package='neo_relayboard_v3', 
          executable='relayboardv3_node',
          output='screen',
          name='neo_relayboard_v3_node',
          parameters = [
              {"pilot_config": "/home/neobotix/ros2_workspace/src/rox/rox_bringup/configs/neo_relayboard_v3/rox-argo/"}
          ]
      )

  # kinematics
  kinematics = Node(
      package='rox_argo_kinematics',
      executable='rox_argo_kinematics_node',
      output='screen',
      name='argo_kinematics_node',
      parameters = [os.path.join(get_package_share_directory('rox_bringup'),'configs/kinematics', rox_type + '_kinematics.yaml')])

  # relay for merging scanners
  relay_topic_lidar1 = Node(
        package='topic_tools',
        executable = 'relay',
        name='relay',
        output='screen',
        parameters=[{'input_topic': "lidar_1/scan_filtered",'output_topic': "scan"}])

  relay_topic_lidar2 = Node(
          package='topic_tools',
          executable = 'relay',
          name='relay',
          output='screen',
          parameters=[{'input_topic': "lidar_2/scan_filtered",'output_topic': "scan"}])


  return [start_robot_state_publisher_cmd, teleop, joy, scan1, scan2, relayboard, kinematics, relay_topic_lidar1, relay_topic_lidar2]

def generate_launch_description():
    opq_function = OpaqueFunction(function=execution_stage, args=[LaunchConfiguration('rox_type',  default="")])

    return LaunchDescription([opq_function])
