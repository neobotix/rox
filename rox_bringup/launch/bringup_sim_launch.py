# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue
import os
from pathlib import Path
import xacro

def execution_stage(context: LaunchContext, 
                    frame_type, 
                    rox_type, 
                    arm_type, 
                    d435_enable, 
                    imu_enable, 
                    ur_dc,
                    initial_joint_controller):
    
    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', 'neo_workshop.sdf')
    bridge_config_file = os.path.join(get_package_share_directory('rox_bringup'), 'configs/gz_bridge', 'gz_bridge_config.yaml')
    frame_typ = str(frame_type.perform(context))
    arm_typ = str(arm_type.perform(context))
    rox_typ = str(rox_type.perform(context))
    d435 = str(d435_enable.perform(context))
    imu = str(imu_enable.perform(context))
    use_ur_dc = str(ur_dc.perform(context))
    initial_joint_controller_name = str(initial_joint_controller.perform(context))
    joint_type = "fixed"

    if (rox_typ == "meca"):
        frame_typ = "long"
        print("Meca only supports long frame")

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
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        )
        , launch_arguments={'gz_args': ['-r ', default_world_path]}.items()
      )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Pass use_sim_time as True for simulation
            'robot_description': ParameterValue(Command([
            "xacro", " ", urdf, " ", 'frame_type:=',
            frame_typ,
            " ", 'arm_type:=',
            arm_typ,
            " ", 'rox_type:=',
            rox_typ,
            " ", 'joint_type:=',
            joint_type,
            " ", 'use_imu:=',
            imu,
            " ", 'd435_enable:=',
            d435,
            " ", 'use_gz:=',
            "True",
            " ", 'use_ur_dc:=',
            use_ur_dc,
            " ", 'force_abs_paths:=',
            "True"  # Pass force_abs_paths as True for simulation
            ]), value_type=str)}],
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
        parameters=[{'config_file': bridge_config_file}])

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

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('neo_gz_worlds'), 'models') + ':' +
        # Use dirname to include the parent directory of rox_description
        os.path.dirname(get_package_share_directory('rox_description'))
    )

    launch_actions = [set_env_vars_resources, start_robot_state_publisher_cmd, gz_sim, gz_bridge, teleop, spawn_robot]

    if arm_typ != '':
        launch_actions.append(joint_state_broadcaster_spawner)
        launch_actions.append(initial_joint_controller_spawner_started)

    return launch_actions

def generate_launch_description():
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
            description='Arm Types:\n'
                        '\t Elite Arms: ec66, cs66\n'
                        '\t Universal Robotics (UR): ur5, ur10, ur5e, ur10e' 
        )

    declare_ur_pwr_variant_cmd = DeclareLaunchArgument(
            'use_ur_dc', default_value='false',
            description='Set this argument to True if you have an UR arm with DC variant'
        )

    declare_initial_joint_controller_cmd = DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='scaled_joint_trajectory_controller',
            description='Robot controller to start:\n'
                        '\t Elite Arms: arm_controller\n'
                        '\t Universal Robotics (UR): joint_trajectory_controller,scaled_joint_trajectory_controller'
        )

    opq_function = OpaqueFunction(function=execution_stage,
                                  args=[LaunchConfiguration('frame_type'),
                                        LaunchConfiguration('rox_type'),
                                        LaunchConfiguration('arm_type'),
                                        LaunchConfiguration('d435_enable'),
                                        LaunchConfiguration('imu_enable'),
                                        LaunchConfiguration('use_ur_dc'),
                                        LaunchConfiguration('initial_joint_controller')
                                        ])
    
    ld = LaunchDescription([
        declare_imu_cmd,
        declare_realsense_cmd,
        declare_arm_cmd,
        declare_frame_type_cmd,
        declare_rox_type_cmd,
        declare_ur_pwr_variant_cmd,
        declare_initial_joint_controller_cmd,
        opq_function
    ])
    return ld
