from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("fake", default_value="true"),
        DeclareLaunchArgument("rws_ip", default_value="192.168.125.1"),
        DeclareLaunchArgument("rws_port", default_value="443"),
        DeclareLaunchArgument("no_connection_timeout", default_value="false"),
        DeclareLaunchArgument("polling_rate", default_value="5.0", description="The frequency [Hz] at which the controller state is collected."),
    ]

    # Launch configurations
    use_fake_hardware = LaunchConfiguration("fake")
    rws_ip = LaunchConfiguration("rws_ip")
    rws_port = LaunchConfiguration("rws_port")
    polling_rate = LaunchConfiguration("polling_rate")
    no_connection_timeout = LaunchConfiguration("no_connection_timeout")

    # Robot description content
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("drims2_description"), "urdf", "yumi", "yumi_cell.urdf.xacro"]),
        " use_fake_hardware:=", use_fake_hardware,
        " rws_ip:=", rws_ip,
        " rws_port:=", rws_port,
        " configure_via_rws:=true"
    ])

    # Convert robot_description to a string to avoid YAML parsing issues
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    controllers_config = PathJoinSubstitution([FindPackageShare("drims2_description"), "config","yumi","controllers.yaml"])

    # Controller node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_config,robot_description],
        output="screen",
        condition=IfCondition(use_fake_hardware)
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", 
                "--controller-manager", "/controller_manager"],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", 
                "--controller-manager", "/controller_manager"],
        output='screen',
        condition=IfCondition(use_fake_hardware)
    )

    restart_rapid_node = Node(
        package="abb_utils",
        executable="restart_rapid",
        name="restart_rapid",
        output="screen",
        condition=UnlessCondition(use_fake_hardware)
    )

    control_node_delayed = RegisterEventHandler(
        OnProcessExit(
            target_action=restart_rapid_node,
            on_exit=[controller_manager_node]
        ),
        condition=UnlessCondition(use_fake_hardware)
    )

    # RWS client node
    rws_client_node = Node(
        package="abb_rws_client",
        executable="rws_client",
        name="rws_client",
        output="screen",
        parameters=[
            {"robot_ip": rws_ip},
            {"robot_port": rws_port},
            {"robot_nckname": "yumi"},
            {"polling_rate": polling_rate},
            {"no_connection_timeout": no_connection_timeout},
        ],
        condition=UnlessCondition(use_fake_hardware)
    )

    smart_gripper_action_control = Node(
        package="smart_gripper_action_control",
        executable="smart_gripper_action_control",
        name="smart_gripper_action_control",
        output="screen",
        namespace="smart_gripper_action_control",
        condition=UnlessCondition(use_fake_hardware)
    )

    nodes = [
        restart_rapid_node,
        control_node_delayed,
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        gripper_controller_spawner,
        smart_gripper_action_control,
        rws_client_node 
    ]

    return LaunchDescription(declared_arguments + nodes)
