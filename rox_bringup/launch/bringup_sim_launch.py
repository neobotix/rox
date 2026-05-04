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
                    arm2_type,
                    imu_enable,
                    d435_enable,
                    scanner_type,
                    ur_dc,
                    # gripper_type,
                    headless_sim,
                    use_wall_time,
                    enable_linear_axis):

    launch_actions = []

    rox_typ = str(rox_type.perform(context))
    arm_typ = str(arm_type.perform(context))
    arm2_typ = str(arm2_type.perform(context))
    # gripper_typ = str(gripper_type.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    d435 = str(d435_enable.perform(context))
    imu = str(imu_enable.perform(context))
    use_ur_dc = str(ur_dc.perform(context))
    headless_sim = str(headless_sim.perform(context)).lower()
    use_wall_time = str(use_wall_time.perform(context)) in ('true', 'True')
    enable_la = str(enable_linear_axis.perform(context)).lower() in ('true',)
    joint_type = "fixed"

    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', 'neo_workshop.sdf')
    bridge_config_file = os.path.join(get_package_share_directory('rox_bringup'), 'configs/gz_bridge', 'gz_bridge_config.yaml')

    # include_gripper_ros2_control = "false"
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

    # Simulation Controllers for the arm(s) and linear axis
    # All controllers must be in a SINGLE YAML because there is only one gz_ros2_control plugin
    arm_manufacturer = None
    arm2_manufacturer = None
    initial_joint_controller_name = "joint_trajectory_controller"
    initial_arm2_joint_controller_name = "arm2_joint_trajectory_controller"
    simulation_controllers = ""
    # initial_gripper_controller_name = ""

    if arm_typ:
        include_arm_ros2_control = "true"
        if arm_typ in ['ec66', 'cs66']:
            arm_manufacturer = 'elite'
            initial_joint_controller_name = 'arm_controller'
        elif arm_typ in ['ur5', 'ur10', 'ur5e', 'ur10e']:
            arm_manufacturer = 'ur'

    if arm2_typ:
        include_arm_ros2_control = "true"
        if arm2_typ in ['ur5', 'ur10', 'ur5e', 'ur10e']:
            arm2_manufacturer = 'ur'

    # Select the appropriate simulation controllers YAML
    controllers_yaml = None

    if arm_typ and arm2_typ and arm_manufacturer == 'ur' and arm2_manufacturer == 'ur':
        # Dual UR arm config already includes linear axis controllers
        controllers_yaml = os.path.join(
            get_package_share_directory('rox_bringup'),
            'configs', 'ur', 'simulation_controllers_dual.yaml'
        )
    elif arm_typ and arm_manufacturer:
        # Single arm setup
        arm_controllers_yaml = os.path.join(
            get_package_share_directory('rox_bringup'),
            'configs', arm_manufacturer, 'simulation_controllers.yaml'
        )
        if enable_la:
            # Merge single arm + linear axis configs at runtime
            import yaml
            linear_axis_controllers_yaml = os.path.join(
                get_package_share_directory('rox_bringup'),
                'configs', 'linear_axis', 'simulation_controllers.yaml'
            )
            with open(arm_controllers_yaml, 'r') as f:
                arm_cfg = yaml.safe_load(f)
            with open(linear_axis_controllers_yaml, 'r') as f:
                la_cfg = yaml.safe_load(f)

            # Merge controller_manager entries
            if 'controller_manager' in la_cfg:
                arm_cfg.setdefault('controller_manager', {}).setdefault('ros__parameters', {}).update(
                    la_cfg['controller_manager'].get('ros__parameters', {})
                )
            # Merge top-level controller parameter blocks
            for key, value in la_cfg.items():
                if key != 'controller_manager':
                    arm_cfg[key] = value

            merged_yaml_path = os.path.join(Path.home(), '.ros', 'simulation_controllers_merged.yaml')
            os.makedirs(os.path.dirname(merged_yaml_path), exist_ok=True)
            with open(merged_yaml_path, 'w') as f:
                yaml.dump(arm_cfg, f, default_flow_style=False)
            controllers_yaml = merged_yaml_path
        else:
            controllers_yaml = arm_controllers_yaml
    elif enable_la:
        # Linear axis only (no arms)
        controllers_yaml = os.path.join(
            get_package_share_directory('rox_bringup'),
            'configs', 'linear_axis', 'simulation_controllers.yaml'
        )

    if controllers_yaml:
        # Generates a final YAML parameter file from the controllers template (with substitutions applied),
        # and returns file_path, shutdown_handler
        simulation_controllers, shutdown_handler = generate_final_yaml(
            context,
            controllers_yaml,
            file_name='simulation_controllers_final.yaml',
            cleanup_enabled=False
        )
        launch_actions.extend(shutdown_handler)

    # if gripper_typ:
    #     include_gripper_ros2_control = "true"
    #     gripper_category = None
    #     if gripper_typ == 'epick':
    #         gripper_category = 'epick'
    #         initial_gripper_controller_name = 'epick_controller'
    #     elif gripper_typ in ['2f_140', '2f_85']:
    #         gripper_category = 'robotiq'
    #         initial_gripper_controller_name = f'robotiq_{gripper_typ}_gripper_controller'
    #     include_gripper_ros2_control = "true"

    xacro_args = [
        "xacro", " ", urdf,
        " ", 'use_gz:=', "true",
        " ", 'rox_type:=', rox_typ,
        " ", 'joint_type:=', joint_type,
        " ", 'use_imu:=', imu,
        " ", 'use_d435:=', d435,
        " ", 'scanner_type:=', scanner_typ,
        " ", 'arm_type:=', arm_typ,
        " ", 'arm2_type:=', arm2_typ,
        # " ", 'gripper_type:=', gripper_typ,
        " ", 'use_ur_dc:=', use_ur_dc,
        " ", 'force_abs_paths:=', "true",
        " ", 'simulation_controllers:=', simulation_controllers,
        " ", 'include_arm_ros2_control:=', include_arm_ros2_control,
        # " ", 'include_gripper_ros2_control:=', include_gripper_ros2_control
        " ", 'enable_linear_axis:=', str(enable_la).lower(),
        " ", 'arm1_prefix:=', "arm1",
        " ", 'arm2_prefix:=', "arm2"
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
        parameters=[{'stamped': False}]  # Set stamped parameter to true for TwistStamped /cmd_vel
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_file, 'override_timestamps_with_wall_time': use_wall_time}]
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

    initial_arm2_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_arm2_joint_controller_name, "-c", "/controller_manager"],
    )

    linear_axis_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_axis_controller", "-c", "/controller_manager"],
    )

    # robotiq_gripper_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[initial_gripper_controller_name, "-c", "/controller_manager"]
    # )

    # Relaying lidar data to /scan topic
    relay_topic_lidar1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar1',
        output='screen',
        parameters=[{
            'input_topic':  '/lidar_1/scan_filtered',
            'output_topic': '/scan'
        }],
    )

    relay_topic_lidar2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar2',
        output='screen',
        parameters=[{
            'input_topic':  '/lidar_2/scan_filtered',
            'output_topic': '/scan'
        }],
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
        # if gripper_typ == 'epick':
        #     env_var_value += ':' + os.path.dirname(get_package_share_directory('epick_description'))
        # else:
        #     env_var_value += ':' + os.path.dirname(get_package_share_directory('robotiq_description'))
            
    set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', env_var_value)

    launch_actions.append(set_env_vars_resources)
    launch_actions.append(start_robot_state_publisher_cmd)
    launch_actions.append(gz_sim)
    launch_actions.append(gz_bridge)
    launch_actions.append(relay_topic_lidar1)
    launch_actions.append(relay_topic_lidar2)
    launch_actions.append(teleop)
    launch_actions.append(spawn_robot)

    # Spawn joint_state_broadcaster if any hardware is present
    if arm_typ or arm2_typ or enable_la:
        launch_actions.append(joint_state_broadcaster_spawner)

    if arm_typ:
        launch_actions.append(initial_joint_controller_spawner_started)
        # if gripper_typ == '2f_140' or gripper_typ == '2f_85':
        #     launch_actions.append(robotiq_gripper_controller_spawner)

    if arm2_typ:
        launch_actions.append(initial_arm2_joint_controller_spawner_started)

    if enable_la:
        launch_actions.append(linear_axis_controller_spawner)

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
            description='Arm 1 Type\n\t'
        )

    declare_arm2_type_cmd = DeclareLaunchArgument(
            'arm2_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e'],
            description='Arm 2 Type\n\t'
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

    declare_use_wall_time_cmd = DeclareLaunchArgument(
            'use_wall_time', default_value='False',
            description='Run gz_bridge with override_timestamps_with_wall_time:=True - Options: True/False'
        )

    declare_enable_linear_axis_cmd = DeclareLaunchArgument(
            'enable_linear_axis', default_value='False',
            description='Enable linear axis (EMROX-Argo variant) - Options: True/False'
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('rox_type'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('arm2_type'),
            LaunchConfiguration('imu_enable'),
            LaunchConfiguration('d435_enable'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('use_ur_dc'),
            # LaunchConfiguration('gripper_type'),
            LaunchConfiguration('headless_simulation'),
            LaunchConfiguration('use_wall_time'),
            LaunchConfiguration('enable_linear_axis')
            ])

    ld = LaunchDescription([
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_scanner_cmd,
        declare_arm_type_cmd,
        declare_arm2_type_cmd,
        declare_rox_type_cmd,
        declare_ur_pwr_variant_cmd,
        # declare_gripper_type_cmd,
        declare_headless_sim_cmd,
        declare_use_wall_time_cmd,
        declare_enable_linear_axis_cmd,
        opq_function
    ])
    return ld
