# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('rox_rviz'),
        'configs',
        'rox.rviz')

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')

    return LaunchDescription([rviz])
