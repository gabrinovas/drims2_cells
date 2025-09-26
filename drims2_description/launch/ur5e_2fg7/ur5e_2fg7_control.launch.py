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
        parameters=[controllers_config,robot_description],
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

    # Replace the onrobot_fake_controller_spawner section with:
    onrobot_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["onrobot_2fg7_gripper_controller", 
                "--controller-manager", "/controller_manager"],
        output='screen',
        # Add a delay to ensure the hardware is ready
        condition=IfCondition(LaunchConfiguration("fake"))
    )

    # Add a timer for the gripper controller to ensure proper startup sequence
    delayed_gripper_controller = TimerAction(
        period=3.0,
        actions=[onrobot_gripper_controller_spawner]
    )

    # OnRobot 2FG7 Driver for real hardware
    onrobot_driver_node = Node(
        package='onrobot_2fg7_driver',
        executable='onrobot_2fg7_driver_node',
        name='onrobot_2fg7_driver',
        output='screen',
        parameters=[{
            'robotkit_ip': '192.168.1.100',  # Your RobotKit IP
            'port': 80,
            'timeout': 2.0,
        }],
        condition=UnlessCondition(LaunchConfiguration('fake'))
    )

    what_to_launch = [
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        robot_state_publisher_node,
        delayed_gripper_controller,
        onrobot_driver_node
    ]

    return what_to_launch

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"))
    launch_args.append(DeclareLaunchArgument(name="robot_ip", default_value="0.0.0.0", description="Robot ip"))

    ld = LaunchDescription(launch_args+[OpaqueFunction(function=launch_setup)])

    return ld