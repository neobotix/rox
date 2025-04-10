# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch_ros.descriptions import ParameterValue
import os
from pathlib import Path
import xacro

def execution_stage(context: LaunchContext, rox_type, arm_type, ur_dc):
    arm_typ = arm_type.perform(context)
    rox_typ = rox_type.perform(context)
    use_ur_dc = ur_dc.perform(context)

    urdf = os.path.join(get_package_share_directory('rox_description'), 'urdf', 'rox.urdf.xacro')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    "xacro", " ", urdf,
                    " ", 'arm_type:=', arm_typ,
                    " ", 'rox_type:=', rox_typ,
                    " ", 'use_ur_dc:=', use_ur_dc,
                ]), 
                value_type=str
            )
        }]
    )

    return [start_robot_state_publisher_cmd]

def generate_launch_description():

    # Launch configuration
    rox_type = LaunchConfiguration('rox_type')
    arm_type = LaunchConfiguration('arm_type')
    ur_dc = LaunchConfiguration('use_ur_dc')

    declare_rox_type_cmd = DeclareLaunchArgument(
            'rox_type', default_value='argo',
            choices = ['', 'argo', 'argo-trio', 'diff', 'trike'],
            description='ROX Drive Type\n\t'
        )

    declare_arm_type_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e', 'ec66', 'cs66'],
            description='Arm Types\n\t'        
        )

    declare_ur_pwr_variant_cmd = DeclareLaunchArgument(
            'use_ur_dc', default_value='False',
            description='Set this argument to True if you have an UR arm with DC variant'
        )

    context_arguments = [rox_type, arm_type, ur_dc]

    opq_function = OpaqueFunction(function=execution_stage,
                                  args=context_arguments)

    ld = LaunchDescription()
    ld.add_action(declare_rox_type_cmd)
    ld.add_action(declare_arm_type_cmd)
    ld.add_action(declare_ur_pwr_variant_cmd)
    ld.add_action(opq_function)

    return ld
