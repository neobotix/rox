# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext
from launch.conditions import UnlessCondition
import os
from pathlib import Path
import xacro

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    rox_type,
                    arm_type,
                    arm2_type,
                    scanner_type,
                    use_imu,
                    ur_dc,
                    mock_arm,
                    initial_controller_arm,
                    robot_ip_arm1,
                    robot_ip_arm2,
                    controllers_yaml,
                    gripper_type,
                    ioboard,
                    enable_linear_axis,
                    arm1_prefix,
                    arm2_prefix):

    rox = get_package_share_directory('rox_bringup')
    
    rox_typ = str(rox_type.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    imu_enable = str(use_imu.perform(context))
    ioboard_enable = str(ioboard.perform(context))
    enable_la = str(enable_linear_axis.perform(context))

    # Manipulator launch arguments
    arm_typ = str(arm_type.perform(context))
    arm2_typ = str(arm2_type.perform(context))
    use_ur_dc = str(ur_dc.perform(context))
    use_mock = str(mock_arm.perform(context))
    gripper_typ = str(gripper_type.perform(context))
    initial_controller_arm_name = str(initial_controller_arm.perform(context))

    headless_mode = LaunchConfiguration('headless_mode')
    arm1_kinematics = LaunchConfiguration('arm1_kinematics_parameters_file')
    arm2_kinematics = LaunchConfiguration('arm2_kinematics_parameters_file')

    arm1_prefix = str(arm1_prefix.perform(context))
    arm2_prefix = str(arm2_prefix.perform(context))

    launch_actions = []

    joint_type = "revolute"
    if use_mock.lower() == "true":
        joint_type = "fixed"

    rp_ns = ""
    if (robot_namespace.perform(context) != "/"):
        rp_ns = robot_namespace.perform(context) + "/"

    urdf = os.path.join(get_package_share_directory('rox_description'), 
                        'urdf', 
                        'rox.urdf.xacro')

    xacro_args = [
        "xacro", " ", urdf,
        " ", 'rox_type:=', rox_typ,
        " ", 'arm_type:=', arm_typ,
        " ", 'arm2_type:=', arm2_typ,
        " ", 'robot_ip_arm1:=', robot_ip_arm1,
        " ", 'robot_ip_arm2:=', robot_ip_arm2,
        " ", 'gripper_type:=', gripper_typ,
        " ", 'use_mock_hardware:=', use_mock,
        " ", 'mock_sensor_commands:=', use_mock,
        " ", 'use_mock_linear_axis:=', LaunchConfiguration('use_mock_linear_axis'),
        " ", 'scanner_type:=', scanner_typ,
        " ", 'use_imu:=', imu_enable,
        " ", 'use_ur_dc:=', use_ur_dc,
        " ", 'joint_type:=', joint_type,
        " ", 'enable_linear_axis:=', enable_la,
        " ", 'arm1_prefix:=', arm1_prefix,
        " ", 'arm2_prefix:=', arm2_prefix,
        " ", 'headless_mode:=', headless_mode,
        " ", 'script_filename:=', LaunchConfiguration('ur_script_filename'),
        " ", 'arm1_kinematics_parameters_file:=', arm1_kinematics,
        " ", 'arm2_kinematics_parameters_file:=', arm2_kinematics,
        " ", 'arm1_reverse_port:=', LaunchConfiguration('arm1_reverse_port'),
        " ", 'arm1_script_sender_port:=', LaunchConfiguration('arm1_script_sender_port'),
        " ", 'arm1_script_command_port:=', LaunchConfiguration('arm1_script_command_port'),
        " ", 'arm1_trajectory_port:=', LaunchConfiguration('arm1_trajectory_port'),
        " ", 'arm2_reverse_port:=', LaunchConfiguration('arm2_reverse_port'),
        " ", 'arm2_script_sender_port:=', LaunchConfiguration('arm2_script_sender_port'),
        " ", 'arm2_script_command_port:=', LaunchConfiguration('arm2_script_command_port'),
        " ", 'arm2_trajectory_port:=', LaunchConfiguration('arm2_trajectory_port')
    ]
    if arm_typ != "":
        xacro_args.extend([" include_arm_ros2_control:=", "true"]) # Include only the arm ros2_control tags
    if arm2_typ != "":
        xacro_args.extend([" include_arm_ros2_control:=", "true"]) # Include only the arm ros2_control tags

    # If user wants to deliberately set it to True, then they have to change it manually in the configs
    if ioboard_enable.lower() == 'true':
        file_path = (
            "/home/neobotix/ros2_workspace/src/rox/rox_bringup/configs/"
            "neo_relayboard_v3/generic/RelayBoardV3Node/remote_config/enable_io_board"
        )
        with open(file_path, "w") as file:
            file.write("true")

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(xacro_args), value_type=str),
            'frame_prefix': rp_ns
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ],
    )

    launch_actions.append(start_robot_state_publisher_cmd)

    #  Launch hardware nodes
    # 1. Relayboard
    relayboard = Node(
        package='neo_relayboard_v3', 
        executable='relayboardv3_node',
        output='screen',
        name='neo_relayboard_v3_node',
        parameters = [os.path.join(rox,'configs/neo_relayboard_v3', 'relayboard_v3.yaml')],
        condition=UnlessCondition(mock_arm)
    )

    launch_actions.append(relayboard)
                    
    # 2. Kinematics
    if (rox_typ == "argo" or rox_typ == "argo-trio"):
        kinematics = Node(
            package='rox_argo_kinematics',
            executable='rox_argo_kinematics_node',
            output='screen',
            name='argo_kinematics_node',
            parameters = [os.path.join(rox,'configs/kinematics', f'{rox_typ}_kinematics.yaml')],
            condition=UnlessCondition(mock_arm)
        )

        launch_actions.append(kinematics)

    if (rox_typ == "diff" or rox_typ == "trike"):
        kinematics = Node(
            package='rox_diff_kinematics',
            executable='rox_diff_kinematics_node',
            output='screen',
            name='diff_kinematics_node',
            parameters = [os.path.join(rox,'configs/kinematics', f'{rox_typ}_kinematics.yaml')],
            condition=UnlessCondition(mock_arm)
        )

        launch_actions.append(kinematics)

    # 3. Teleop
    teleop = Node(
        package='neo_teleop2',
        executable='neo_teleop2_node',
        output='screen',
        name='neo_teleop2_node',
        parameters = [os.path.join(rox,'configs/teleop', f'{rox_typ}_teleop.yaml')],
        condition=UnlessCondition(mock_arm)
    )

    launch_actions.append(teleop)

    # Joy
    joy = Node(
            package='joy', 
            executable='joy_node', 
            output='screen',
            name='joy_node',
            parameters = [{'dev': "/dev/input/js0"}, {'deadzone':0.12}],
            condition=UnlessCondition(mock_arm)
        )

    launch_actions.append(joy)

    # 4. Laser - Nanoscan
    if scanner_typ == "nanoscan":
        scan1 = Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="lidar_1_node",
                output="screen",
                emulate_tty=True,
                parameters=[os.path.join(rox, 
                                'configs/sick_lidar', 
                                'nanoscan_1.yaml')],
                condition=UnlessCondition(mock_arm),
                remappings=[
                    ('/scan', '/lidar_1/scan_filtered'),
                    ('/extended_scan', '/lidar_1/extended_scan'),
                    ('/output_paths', '/lidar_1/output_paths'),
                    ('/raw_data', '/lidar_1/raw_data'),
                    ('/field_data', '/lidar_1/field_data'),
                    ('/diagnostics', '/lidar_1/diagnostics')
                ]
            )

        launch_actions.append(scan1)

        scan2 = Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name="lidar_2_node",
                output="screen",
                emulate_tty=True,
                parameters=[os.path.join(rox, 
                                'configs/sick_lidar', 
                                'nanoscan_2.yaml')],
                condition=UnlessCondition(mock_arm),
                remappings=[
                    ('/scan', '/lidar_2/scan_filtered'),
                    ('/extended_scan', '/lidar_2/extended_scan'),
                    ('/output_paths', '/lidar_2/output_paths'),
                    ('/raw_data', '/lidar_2/raw_data'),
                    ('/field_data', '/lidar_2/field_data'),
                    ('/diagnostics', '/lidar_2/diagnostics')
                ]
            )

        launch_actions.append(scan2)

    # Laser - PsenScan
    elif scanner_typ == "psenscan":
        scan = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(get_package_share_directory('psen_scan_v2'),
                    'launch',
                    'psen_scan_v2.launch.xml')
            ),
            condition=UnlessCondition(mock_arm),
            launch_arguments={
                'sensor_ip': "192.168.1.30",
                'host_ip': "192.168.1.10"
            }.items()
        )

        launch_actions.append(scan)

    # 5. IMU
    if imu_enable.lower == 'true':
        imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rox, 
                    'configs/phidget_imu', 
                    'imu_launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=UnlessCondition(mock_arm)
        )

        launch_actions.append(imu)

    # 7. Arm - Bringing up drivers for Universal Arm
    # TODO: Add support for Elite Robots
    # TODO: Add support for namespacing
    ur_types = ("ur5", "ur10", "ur5e", "ur10e")

    if arm_typ in ur_types and arm2_typ in ur_types:
        arm1_initial_controller = f"arm1_{initial_controller_arm_name}"
        arm2_initial_controller = f"arm2_{initial_controller_arm_name}"
        if use_mock.lower() == 'true':
            arm1_initial_controller = "arm1_joint_trajectory_controller"
            arm2_initial_controller = "arm2_joint_trajectory_controller"

        dual_ur_arms = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rox, 'configs/ur', 'ur_dual_control.launch.py')
            ),
            launch_arguments={
                'arm1_ur_type': arm_typ,
                'arm2_ur_type': arm2_typ,
                'robot_ip_arm1': robot_ip_arm1,
                'robot_ip_arm2': robot_ip_arm2,
                'use_mock_hardware': mock_arm,
                'headless_mode': headless_mode,
                'arm1_initial_joint_controller': arm1_initial_controller,
                'arm2_initial_joint_controller': arm2_initial_controller,
                'controllers_file': LaunchConfiguration('dual_controllers_file'),
                'enable_linear_axis': enable_la,
                'arm1_prefix': arm1_prefix,
                'arm2_prefix': arm2_prefix,
                'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
            }.items(),
        )
        launch_actions.append(dual_ur_arms)

    elif (arm_typ == "ur5" or
        arm_typ == "ur10" or
        arm_typ == "ur5e" or
        arm_typ == "ur10e"):

        # Mock hardware supports only `joint_trajectory_controller`
        if use_mock.lower() == 'true':
            initial_controller_arm_name = "joint_trajectory_controller"
            
        ur_arm = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rox,
                        'configs/ur',
                        'ur_control.launch.py') 
                ),
                launch_arguments={
                    'ur_type': arm_typ,
                    'robot_ip': robot_ip_arm1,
                    'tf_prefix': arm1_prefix,
                    'use_mock_hardware': mock_arm,
                    'mock_sensor_commands': mock_arm,
                    'initial_joint_controller': initial_controller_arm_name,
                    'controllers_file': controllers_yaml,
                    'enable_linear_axis': enable_la,
                    'headless_mode': headless_mode,
                    'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
                }.items()
            )

        launch_actions.append(ur_arm)

        # Conditionally add grippers
        if gripper_typ != "":
            gripper_xacro_args = xacro_args.copy()
            gripper_xacro_args.extend([
                " include_gripper_ros2_control:=", "true",# Include only the gripper ros2_control tags
                " include_arm_ros2_control:=", "false"])

            # For 2f_140
            if (gripper_typ == "2f_140" or gripper_typ == "2f_85"):
                robotiq_2f_gripper = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(rox,
                                'configs/robotiq',
                                'robotiq_control.launch.py')
                        ),
                        launch_arguments={
                            'robot_description_content': Command(gripper_xacro_args),
                            'use_mock_hardware': use_mock,
                            'model': os.path.join(get_package_share_directory('robotiq_description'),
                                            "urdf", 
                                            f"robotiq_{gripper_typ}_gripper.urdf.xacro"
                                            ),
                            'controllers_file': f"robotiq_{gripper_typ}_controllers.yaml"
                        }.items()
                    )

                launch_actions.append(robotiq_2f_gripper)

            # For Epick
            elif (gripper_typ == "epick"):
                gripper_epick = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(rox,
                                'configs/robotiq',
                                'robotiq_epick_control.launch.py')
                        )
                    )

                launch_actions.append(gripper_epick)

    # Relaying lidar data to /scan topic
    relay_topic_lidar1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar1',
        namespace=rp_ns,
        output='screen',
        parameters=[{'input_topic': rp_ns + "lidar_1/scan_filtered",'output_topic': rp_ns + "scan"}],
        condition=UnlessCondition(mock_arm)
    )

    relay_topic_lidar2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar2',
        namespace=rp_ns,
        output='screen',
        parameters=[{'input_topic': rp_ns + "lidar_2/scan_filtered",'output_topic': rp_ns + "scan"}],
        condition=UnlessCondition(mock_arm)
    )

    # Relay drive/joint_states topic to joint_states
    relay_topic_joint_states = Node(
        package='topic_tools',
        executable='relay',
        name='relay_joint_states',
        output='screen',
        parameters=[{'input_topic': "drive/joint_states",'output_topic': "joint_states"}],
        condition=UnlessCondition(mock_arm)
    )

    launch_actions.append(relay_topic_lidar1)
    launch_actions.append(relay_topic_lidar2)
    launch_actions.append(relay_topic_joint_states)

    return launch_actions

def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', description='Top-level namespace'
        )

    declare_rox_type_cmd = DeclareLaunchArgument(
            'rox_type', default_value='argo',
            choices = ['', 'argo', 'argo-trio', 'diff', 'trike'],
            description='Robot type\n\t'
        )

    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )

    declare_scanner_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='nanoscan',
            description='Scanner options available: nanoscan/psenscan'
        )

    declare_arm_cmd = DeclareLaunchArgument(
            'arm_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e'],
            description='Arm used in the robot - currently only Universal Robotics arms are supported\n\t'
        )
    
    declare_arm2_cmd = DeclareLaunchArgument(
            'arm2_type', default_value='',
            choices=['', 'ur5', 'ur10', 'ur5e', 'ur10e'],
            description='Arm used in the robot - currently only Universal Robotics arms are supported\n\t'
        )

    declare_ur_pwr_variant_cmd = DeclareLaunchArgument(
            'use_ur_dc', default_value='False',
            description='Set this argument to True if you have an UR arm with DC variant'
        )

    declare_mock_arm_cmd = DeclareLaunchArgument(
            'use_mock_arm', default_value='False',
            description="Mock arm and gripper (if available)"
        )

    declare_mock_linear_axis_cmd = DeclareLaunchArgument(
            'use_mock_linear_axis', default_value=LaunchConfiguration('use_mock_arm'),
            description='Use mock ros2_control hardware for the linear axis independently.'
        )

    declare_initial_controller_arm_cmd = DeclareLaunchArgument(
            'initial_controller_arm', default_value='scaled_joint_trajectory_controller',
            choices=['', 'joint_trajectory_controller', 'scaled_joint_trajectory_controller'],
            description='Initial controller for the arm\n\t'
        )

    declare_robot_ip_arm1_cmd = DeclareLaunchArgument(
            'robot_ip_arm1', default_value='192.168.1.102',
            description='IP address of the robot arm1.'
        )

    declare_robot_ip_arm2_cmd = DeclareLaunchArgument(
            'robot_ip_arm2', default_value='192.168.1.103',
            description='IP address of the robot arm2.'
        )

    declare_controllers_file_cmd = DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(
                get_package_share_directory('rox_bringup'),
                'configs/ur/ur_controllers.yaml'
            ),
            description='YAML file with the arm controllers configuration.',
        )

    declare_dual_controllers_file_cmd = DeclareLaunchArgument(
        'dual_controllers_file',
        default_value=os.path.join(
            get_package_share_directory('rox_bringup'),
            'configs/ur/ur_controllers_dual.yaml'
        ),
        description='YAML file containing controllers for both UR arms.',
    )

    declare_headless_mode_cmd = DeclareLaunchArgument(
        'headless_mode', default_value='False',
        description='Run both UR drivers in headless mode.'
    )

    declare_controller_spawner_timeout_cmd = DeclareLaunchArgument(
        'controller_spawner_timeout', default_value='60',
        description='Seconds controller spawners wait while all hardware initializes.'
    )

    declare_ur_script_filename_cmd = DeclareLaunchArgument(
        'ur_script_filename',
        default_value=PathJoinSubstitution([
            '/opt/ros', EnvironmentVariable('ROS_DISTRO'), 'share',
            'ur_client_library', 'resources', 'external_control.urscript'
        ]),
        description='External-control URScript matching the installed ur_robot_driver binaries.'
    )

    declare_arm1_kinematics_cmd = DeclareLaunchArgument(
        'arm1_kinematics_parameters_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config',
            LaunchConfiguration('arm_type'), 'default_kinematics.yaml'
        ]),
        description='Calibration/kinematics YAML for arm 1.'
    )

    declare_arm2_kinematics_cmd = DeclareLaunchArgument(
        'arm2_kinematics_parameters_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config',
            LaunchConfiguration('arm2_type'), 'default_kinematics.yaml'
        ]),
        description='Calibration/kinematics YAML for arm 2.'
    )

    port_arguments = [
        DeclareLaunchArgument('arm1_reverse_port', default_value='50001'),
        DeclareLaunchArgument('arm1_script_sender_port', default_value='50002'),
        DeclareLaunchArgument('arm1_script_command_port', default_value='50004'),
        DeclareLaunchArgument('arm1_trajectory_port', default_value='50003'),
        DeclareLaunchArgument('arm2_reverse_port', default_value='50006'),
        DeclareLaunchArgument('arm2_script_sender_port', default_value='50007'),
        DeclareLaunchArgument('arm2_script_command_port', default_value='50010'),
        DeclareLaunchArgument('arm2_trajectory_port', default_value='50009'),
    ]

    declare_robotiq_cmd = DeclareLaunchArgument(
            'gripper_type', default_value='',
            choices=['', '2f_140', '2f_85', 'epick'],
            description="Enables gripper and it's controllers"
        )

    declare_enable_ioboard = DeclareLaunchArgument(
            'enable_io_board', default_value='False',
            choices=['True', 'False'],
            description="Enables or Disables IOBoard if present - might require restart of the robot"
        )

    declare_enable_linear_axis = DeclareLaunchArgument(
            'enable_linear_axis', default_value='True',
            choices=['True', 'False'],
            description="Enables or Disables Linear Axis if present - might require restart of the robot"
        )

    declare_arm1_prefix = DeclareLaunchArgument(
            'arm1_prefix', default_value='arm1_',
            description="Prefix for the first arm"
        )

    declare_arm2_prefix = DeclareLaunchArgument(
            'arm2_prefix', default_value='arm2_',
            description="Prefix for the second arm"
        )

    opq_function = OpaqueFunction(
        function=execution_stage,
        args=[
            LaunchConfiguration('robot_namespace'),
            LaunchConfiguration('rox_type'),
            LaunchConfiguration('arm_type'),
            LaunchConfiguration('arm2_type'),
            LaunchConfiguration('scanner_type'),
            LaunchConfiguration('imu_enable'),
            LaunchConfiguration('use_ur_dc'),
            LaunchConfiguration('use_mock_arm'),
            LaunchConfiguration('initial_controller_arm'),
            LaunchConfiguration('robot_ip_arm1'),
            LaunchConfiguration('robot_ip_arm2'),
            LaunchConfiguration('controllers_file'),
            LaunchConfiguration('gripper_type'),
            LaunchConfiguration('enable_io_board'),
            LaunchConfiguration('enable_linear_axis'),
            LaunchConfiguration('arm1_prefix'),
            LaunchConfiguration('arm2_prefix')
            ])  

    ld = LaunchDescription([
        declare_namespace_cmd,
        declare_rox_type_cmd,
        declare_arm_cmd,
        declare_arm2_cmd,
        declare_scanner_cmd,
        declare_imu_cmd,
        declare_ur_pwr_variant_cmd,
        declare_mock_arm_cmd,
        declare_mock_linear_axis_cmd,
        declare_initial_controller_arm_cmd,
        declare_robot_ip_arm1_cmd,
        declare_robot_ip_arm2_cmd,
        declare_controllers_file_cmd,
        declare_dual_controllers_file_cmd,
        declare_headless_mode_cmd,
        declare_controller_spawner_timeout_cmd,
        declare_ur_script_filename_cmd,
        declare_arm1_kinematics_cmd,
        declare_arm2_kinematics_cmd,
        declare_robotiq_cmd,
        declare_enable_ioboard,
        declare_enable_linear_axis,
        declare_arm1_prefix,
        declare_arm2_prefix,
        *port_arguments,
        opq_function
    ])
    return ld
