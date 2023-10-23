# Neobotix GmbH

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():

    urdf = os.path.join(get_package_share_directory('rox_description'), 'urdf', 'rox.urdf')

    with open(urdf, 'r') as infp:  
        robot_desc = infp.read()

    # ToDo: Namespacing
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
		arguments=[urdf])

    return LaunchDescription([start_robot_state_publisher_cmd])
