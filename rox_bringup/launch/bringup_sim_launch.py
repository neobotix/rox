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
                    imu_enable,
                    d435_enable,
                    scanner_type,
                    ur_dc,
                    gripper_type,
                    headless_sim):

    launch_actions = []

    rox_typ = str(rox_type.perform(context))
    arm_typ = str(arm_type.perform(context))
    gripper_typ = str(gripper_type.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    d435 = str(d435_enable.perform(context))
    imu = str(imu_enable.perform(context))
    use_ur_dc = str(ur_dc.perform(context))
    headless_sim = str(headless_sim.perform(context)).lower()
    joint_type = "fixed"

    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', 'neo_workshop.sdf')
    bridge_config_file = os.path.join(get_package_share_directory('rox_bringup'), 'configs/gz_bridge', 'gz_bridge_config.yaml')

    include_gripper_ros2_control = "false"
    include_arm_ros2_control = "false"

    if (rox_typ == "diff" or rox_typ == "trike"):
        joint_type = "revolute"

    # Getting the robot description xacro
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

    # Simulation Controllers for the arm
    arm_manufacturer = None
    initial_joint_controller_name = "joint_trajectory_controller"
    initial_gripper_controller_name = ""
    if arm_typ:
        include_arm_ros2_control = "true"
        if arm_typ in ['ec66', 'cs66']:
            arm_manufacturer = 'elite'
            initial_joint_controller_name = 'arm_controller'
        elif arm_typ in ['ur5', 'ur10', 'ur5e', 'ur10e']:
            arm_manufacturer = 'ur'

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

        if gripper_typ:
            include_gripper_ros2_control = "true"
            gripper_category = None
            if gripper_typ == 'epick':
                gripper_category = 'epick'
                initial_gripper_controller_name = 'epick_controller'
            elif gripper_typ in ['2f_140', '2f_85']:
                gripper_category = 'robotiq'
                initial_gripper_controller_name = f'robotiq_{gripper_typ}_gripper_controller'
            include_gripper_ros2_control = "true"

    else:
        simulation_controllers = ""

    xacro_args = [
        "xacro", " ", urdf,
        " ", 'use_gz:=', "true",
        " ", 'rox_type:=', rox_typ,
        " ", 'joint_type:=', joint_type,
        " ", 'use_imu:=', imu,
        " ", 'd435_enable:=', d435,
        " ", 'scanner_type:=', scanner_typ,
        " ", 'arm_type:=', arm_typ,
        " ", 'gripper_type:=', gripper_typ,
        " ", 'use_ur_dc:=', use_ur_dc,
        " ", 'force_abs_paths:=', "true",
        " ", 'simulation_controllers:=', simulation_controllers,
        " ", 'include_arm_ros2_control:=', include_arm_ros2_control,
        " ", 'include_gripper_ros2_control:=', include_gripper_ros2_control
    ]

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Set use_sim_time as True for simulation
            'robot_description': ParameterValue(Command(xacro_args), value_type=str),
        }],
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

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_gripper_controller_name, "-c", "/controller_manager"]
    )

    env_var_value = (
        os.path.join(get_package_share_directory('neo_gz_worlds'), 'models') +
        ':' +
        os.path.dirname(get_package_share_directory('rox_description'))
    )

    if arm_typ:
        # Set environment variable for arm description packages
        if arm_typ == 'ec66' or arm_typ == 'cs66':
            env_var_value += ':' + os.path.dirname(get_package_share_directory('elite_description'))
        elif arm_typ == 'ur5' or arm_typ == 'ur10' or arm_typ == 'ur5e' or arm_typ == 'ur10e':
            env_var_value += ':' + os.path.dirname(get_package_share_directory('ur_description'))
        # Set environment variable for gripper description packages
        if gripper_typ == 'epick':
            env_var_value += ':' + os.path.dirname(get_package_share_directory('epick_description'))
        else:
            env_var_value += ':' + os.path.dirname(get_package_share_directory('robotiq_description'))
            
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
        if gripper_typ == '2f_140' or gripper_typ == '2f_85':
            launch_actions.append(robotiq_gripper_controller_spawner)

    return launch_actions

def generate_launch_description():

    declare_rox_type_cmd = DeclareLaunchArgument(
            'rox_type',default_value='argo',
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

    declare_scanner_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='nanoscan',
            choices = ['', 'nanoscan', 'psenscan'],
            description='Scanner Type'
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

    declare_gripper_type_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85'],
            description='Gripper Types - Supported Robots [mpo-700, mpo-500]\n\t'
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
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('use_ur_dc'),
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('headless_simulation')
            ])

    ld = LaunchDescription([
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_scanner_cmd,
        declare_arm_type_cmd,
        declare_rox_type_cmd,
        declare_ur_pwr_variant_cmd,
        declare_gripper_type_cmd,
        declare_headless_sim_cmd,
        opq_function
    ])
    return ld
