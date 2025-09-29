from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        " ", 
        PathJoinSubstitution([
            FindPackageShare("drims2_description"), 
            "urdf", "ur5e_2fg7", "ur5e_2fg7_cell.urdf.xacro"
        ]),
        " ", 
        "use_fake_hardware:=", LaunchConfiguration("fake"),
        " ",
        "robot_ip:=", LaunchConfiguration("robot_ip"),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controllers_config = PathJoinSubstitution([FindPackageShare("drims2_description"),
    "config","ur5e_2fg7","controllers.yaml"])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_config, robot_description],
        output="screen"
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", 
                   "--controller-manager", "/controller_manager"],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager","/controller_manager"],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # OnRobot 2FG7 Gripper Controller
    onrobot_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["onrobot_2fg7_gripper_controller", 
                "--controller-manager", "/controller_manager"],
        output='screen',
    )

    # OnRobot Driver Node (separate node for hardware communication)
    onrobot_driver_node = Node(
        package='onrobot_driver',
        executable='onrobot_driver_node',
        name='onrobot_driver',
        output='screen',
        parameters=[{
            'gripper_type': '2FG7',
            'ip_address': '192.168.1.1',
            'port': 502,
            'max_width': 0.085,
            'min_width': 0.0,
            'max_force': 100.0,
            'update_rate': 100.0,
        }],
        condition=UnlessCondition(LaunchConfiguration('fake'))
    )

    # Simple Gripper Command Publisher (for simulation)
    gripper_command_publisher = Node(
        package='onrobot_driver',
        executable='gripper_command_publisher',
        name='gripper_command_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('fake'))  # Only in simulation
    )

    # Add delays for proper startup sequence
    delayed_gripper_controller = TimerAction(
        period=3.0,  # Start gripper controller after basic setup
        actions=[onrobot_gripper_controller_spawner]
    )

    delayed_driver_node = TimerAction(
        period=5.0,  # Start driver after controllers are ready
        actions=[onrobot_driver_node]
    )

    delayed_gripper_publisher = TimerAction(
        period=6.0,  # Start gripper publisher last
        actions=[gripper_command_publisher]
    )

    what_to_launch = [
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        robot_state_publisher_node,
        delayed_driver_node,
        delayed_gripper_controller,
        delayed_gripper_publisher,
    ]

    return what_to_launch

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"))
    launch_args.append(DeclareLaunchArgument(name="robot_ip", default_value="0.0.0.0", description="Robot ip"))

    ld = LaunchDescription(launch_args+[OpaqueFunction(function=launch_setup)])

    return ld