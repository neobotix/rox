# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch_ros.descriptions import ParameterValue
from param_file_utils import generate_final_yaml
import os
from pathlib import Path
import xacro

def execution_stage(context: LaunchContext, 
                    rox_type,
                    arm_type,
                    d435_enable,
                    imu_enable,
                    ur_dc,
                    headless_sim):

    launch_actions = []

    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', 'neo_workshop.sdf')
    bridge_config_file = os.path.join(get_package_share_directory('rox_bringup'), 'configs/gz_bridge', 'gz_bridge_config.yaml')
    arm_typ = str(arm_type.perform(context))
    rox_typ = str(rox_type.perform(context))
    d435 = str(d435_enable.perform(context))
    imu = str(imu_enable.perform(context))
    use_ur_dc = str(ur_dc.perform(context))
    headless_sim = str(headless_sim.perform(context)).lower()
    joint_type = "fixed"

    if (rox_typ == "diff" or rox_typ == "trike"):
        joint_type = "revolute"

    urdf = os.path.join(get_package_share_directory('rox_description'), 'urdf', 'rox.urdf.xacro')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        output='screen',
        arguments=[
            '-topic', "robot_description",
            '-name', "rox"])

    # Define gz_args based on headless_simulation argument
    gz_args = f"-r {default_world_path}"

    if headless_sim == 'true':
        gz_args = f"-r -s {default_world_path}"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ), 
        launch_arguments={'gz_args': gz_args}.items()
      )

    arm_manufacturer = None
    initial_joint_controller_name = "joint_trajectory_controller"
    if arm_typ:
        if arm_typ in ['ec66', 'cs66']:
            arm_manufacturer = 'elite'
            initial_joint_controller_name = 'arm_controller'
        elif arm_typ in ['ur5', 'ur10', 'ur5e', 'ur10e']:
            arm_manufacturer = 'ur'

    if arm_manufacturer is not None:
        controllers_yaml = os.path.join(
            get_package_share_directory('rox_bringup'),
            'configs', 
            arm_manufacturer, 
            'controllers.yaml'
        )

        # Generates a final YAML parameter file from the controllers template (with substitutions applied),
        # and returns file_path, shutdown_handler
        simulation_controllers, shutdown_handler = generate_final_yaml(
            context,
            controllers_yaml,
            file_name='simulation_controllers.yaml',
            cleanup_enabled=False
        )
        launch_actions.extend(shutdown_handler)

    else:
        simulation_controllers = ""

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Set use_sim_time as True for simulation
            'robot_description': ParameterValue(
                Command([
                    "xacro", " ", urdf,
                    " ", 'arm_type:=', arm_typ,
                    " ", 'rox_type:=', rox_typ,
                    " ", 'joint_type:=', joint_type,
                    " ", 'use_imu:=', imu,
                    " ", 'd435_enable:=', d435,
                    " ", 'use_gz:=', "true",
                    " ", 'use_ur_dc:=', use_ur_dc,
                    " ", 'force_abs_paths:=', "true",
                    " ", 'simulation_controllers:=', simulation_controllers
                ]), 
                value_type=str
            )
        }]
    )

    teleop =  Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop',
        parameters=[{'stamped': True}]  # Set stamped parameter to true for TwistStamped /cmd_vel
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_file}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller_name, "-c", "/controller_manager"],
    )

    env_var_value = (
        os.path.join(get_package_share_directory('neo_gz_worlds'), 'models') +
        ':' +
        os.path.dirname(get_package_share_directory('rox_description'))
    )

    if arm_typ == 'ec66' or arm_typ == 'cs66':
        env_var_value += ':' + os.path.dirname(get_package_share_directory('elite_description'))

    # Set environment variable
    set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', env_var_value)

    launch_actions.append(set_env_vars_resources)
    launch_actions.append(start_robot_state_publisher_cmd)
    launch_actions.append(gz_sim)
    launch_actions.append(gz_bridge)
    launch_actions.append(teleop)
    launch_actions.append(spawn_robot)

    if arm_typ != '':
        launch_actions.append(joint_state_broadcaster_spawner)
        launch_actions.append(initial_joint_controller_spawner_started)

    return launch_actions

def generate_launch_description():

    declare_rox_type_cmd = DeclareLaunchArgument(
            'rox_type', default_value='argo',
            choices = ['', 'argo', 'argo-trio', 'diff', 'trike'],
            description='ROX Drive Type\n\t'
        )

    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )

    declare_realsense_cmd = DeclareLaunchArgument(
            'd435_enable', default_value='False',
            description='Enable Realsense - Options: True/False'
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

    declare_headless_sim_cmd = DeclareLaunchArgument(
            'headless_simulation', default_value='False',
            description='Run Gazebo in headless mode (no GUI) - Options: True/False'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('rox_type'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('d435_enable'),
            LaunchConfiguration('imu_enable'),
            LaunchConfiguration('use_ur_dc'),
            LaunchConfiguration('headless_simulation')
            ])

    ld = LaunchDescription([
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_arm_type_cmd,
        declare_rox_type_cmd,
        declare_ur_pwr_variant_cmd,
        declare_headless_sim_cmd,
        opq_function
    ])
    return ld
