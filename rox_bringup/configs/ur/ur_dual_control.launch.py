"""Start two Universal Robots hardware components in one controller manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    arm1_ip = LaunchConfiguration("robot_ip_arm1")
    arm2_ip = LaunchConfiguration("robot_ip_arm2")
    use_mock = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    timeout = LaunchConfiguration("controller_spawner_timeout")
    controllers_file = LaunchConfiguration("controllers_file")
    arm1_controller = LaunchConfiguration("arm1_initial_joint_controller")
    arm2_controller = LaunchConfiguration("arm2_initial_joint_controller")
    activate_arm1 = LaunchConfiguration("arm1_activate_joint_controller")
    activate_arm2 = LaunchConfiguration("arm2_activate_joint_controller")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
    )

    def robot_node(executable, namespace, robot_ip, **kwargs):
        parameters = kwargs.pop("parameters", [{"robot_ip": robot_ip}])
        return Node(
            package="ur_robot_driver",
            executable=executable,
            namespace=namespace,
            output="screen",
            parameters=parameters,
            condition=UnlessCondition(use_mock),
            **kwargs,
        )

    arm1_dashboard = robot_node(
        "dashboard_client", "arm1", arm1_ip, name="dashboard_client", emulate_tty=True
    )
    arm2_dashboard = robot_node(
        "dashboard_client", "arm2", arm2_ip, name="dashboard_client", emulate_tty=True
    )
    arm1_script = robot_node("urscript_interface", "arm1", arm1_ip)
    arm2_script = robot_node("urscript_interface", "arm2", arm2_ip)
    arm1_state_helper = robot_node(
        "robot_state_helper",
        "arm1",
        arm1_ip,
        name="robot_state_helper",
        parameters=[{"headless_mode": headless_mode}, {"robot_ip": arm1_ip}],
    )
    arm2_state_helper = robot_node(
        "robot_state_helper",
        "arm2",
        arm2_ip,
        name="robot_state_helper",
        parameters=[{"headless_mode": headless_mode}, {"robot_ip": arm2_ip}],
    )

    consistent_controllers = [
        "joint_state_broadcaster",
        "arm1_io_and_status_controller",
        "arm2_io_and_status_controller",
        "arm1_speed_scaling_state_broadcaster",
        "arm2_speed_scaling_state_broadcaster",
        "arm1_force_torque_sensor_broadcaster",
        "arm2_force_torque_sensor_broadcaster",
    ]

    def controller_stopper(namespace, activate_controller):
        return Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            namespace=namespace,
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            condition=UnlessCondition(use_mock),
            parameters=[
                {"headless_mode": headless_mode},
                {"joint_controller_active": activate_controller},
                {"consistent_controllers": consistent_controllers},
            ],
        )

    def spawn(controllers, active=True, condition=None):
        arguments = [
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            timeout,
        ]
        if not active:
            arguments.append("--inactive")
        arguments.extend(controllers)
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arguments,
            condition=condition,
        )

    broadcasters = [
        "joint_state_broadcaster",
        "arm1_io_and_status_controller",
        "arm2_io_and_status_controller",
        "arm1_speed_scaling_state_broadcaster",
        "arm2_speed_scaling_state_broadcaster",
        "arm1_force_torque_sensor_broadcaster",
        "arm2_force_torque_sensor_broadcaster",
    ]
    inactive_controllers = [
        "arm1_forward_velocity_controller",
        "arm1_forward_position_controller",
        "arm2_forward_velocity_controller",
        "arm2_forward_position_controller",
    ]
    if LaunchConfiguration("enable_linear_axis").perform(context).lower() == "true":
        inactive_controllers.extend(["linear_axis_controller", "linear_axis_gpio_controller"])

    if activate_arm1.perform(context).lower() == "true":
        broadcasters.append(arm1_controller.perform(context))
    else:
        inactive_controllers.append(arm1_controller.perform(context))

    if activate_arm2.perform(context).lower() == "true":
        broadcasters.append(arm2_controller.perform(context))
    else:
        inactive_controllers.append(arm2_controller.perform(context))

    # ros2_control spawners share a filesystem lock. Starting multiple spawners
    # concurrently makes one process hold that lock while the others time out.
    # Spawn all active controllers together, then load inactive controllers only
    # after the first spawner exits.
    active_spawner = spawn(broadcasters)
    inactive_spawner = spawn(inactive_controllers, active=False)
    start_inactive_after_active = RegisterEventHandler(
        OnProcessExit(target_action=active_spawner, on_exit=[inactive_spawner])
    )

    nodes = [
        control_node,
        arm1_dashboard,
        arm2_dashboard,
        arm1_script,
        arm2_script,
        arm1_state_helper,
        arm2_state_helper,
        controller_stopper("arm1", activate_arm1),
        controller_stopper("arm2", activate_arm2),
        active_spawner,
        start_inactive_after_active,
    ]
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("arm1_ur_type", default_value="ur10e"),
        DeclareLaunchArgument("arm2_ur_type", default_value="ur10e"),
        DeclareLaunchArgument("robot_ip_arm1", default_value="192.168.1.102"),
        DeclareLaunchArgument("robot_ip_arm2", default_value="192.168.1.103"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("enable_linear_axis", default_value="false"),
        DeclareLaunchArgument("arm1_prefix", default_value="arm1_"),
        DeclareLaunchArgument("arm2_prefix", default_value="arm2_"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="60"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rox_bringup"), "configs", "ur", "ur_controllers_dual.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "update_rate_config_file",
            default_value=[
                PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "config"]),
                "/",
                LaunchConfiguration("arm1_ur_type"),
                "_update_rate.yaml",
            ],
        ),
        DeclareLaunchArgument(
            "arm1_initial_joint_controller",
            default_value="arm1_scaled_joint_trajectory_controller",
        ),
        DeclareLaunchArgument(
            "arm2_initial_joint_controller",
            default_value="arm2_scaled_joint_trajectory_controller",
        ),
        DeclareLaunchArgument("arm1_activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("arm2_activate_joint_controller", default_value="true"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
