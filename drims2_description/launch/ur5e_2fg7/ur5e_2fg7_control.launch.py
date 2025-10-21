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

    # Joint state broadcaster - publishes to arm_joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager","/controller_manager"],
        output='screen',
        # Remap the output topic
        remappings=[('/joint_states', '/arm_joint_states')]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # Merged Joint State Publisher Node - from onrobot_driver package
    merged_joint_state_publisher = Node(
        package='onrobot_driver',
        executable='merged_joint_state_publisher',
        name='merged_joint_state_publisher',
        output='screen'
    )

    # OnRobot Driver Node - with delay and publishes to gripper_joint_states
    onrobot_driver_node = TimerAction(
        period=5.0,  # Wait 5 seconds for other systems to initialize
        actions=[
            Node(
                package='onrobot_driver',
                executable='onrobot_driver_node',
                name='onrobot_driver',
                output='screen',
                parameters=[{
                    'gripper_type': '2FG7',
                    'ip_address': LaunchConfiguration('onrobot_ip'),
                    'port': 502,
                    'max_width': 0.070,    # 70mm max opening
                    'min_width': 0.035,    # 35mm min opening
                    'initial_position': 0.055,  # 55mm initial opening
                    'max_force': 140.0,
                    'update_rate': 100.0,
                    'simulation_mode': LaunchConfiguration('fake'),
                    'joint_name': 'left_finger_joint',
                    'command_timeout': 10.0,
                }],
                # CRITICAL: Ensure it publishes to gripper_joint_states
                remappings=[
                    ('/joint_states', '/gripper_joint_states'),  # Driver publishes here
                    ('/gripper_action', '/gripper_action')       # Keep action topic
                ]
            )
        ]
    )

    what_to_launch = [
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        robot_state_publisher_node,
        merged_joint_state_publisher,  # Add the merged publisher
        onrobot_driver_node,  # ONLY driver for gripper (with delay)
    ]
    
    return what_to_launch

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"))
    launch_args.append(DeclareLaunchArgument(name="robot_ip", default_value="0.0.0.0", description="Robot ip"))
    launch_args.append(DeclareLaunchArgument(name="onrobot_ip", default_value="192.168.1.1", description="OnRobot Compute Box IP"))

    ld = LaunchDescription(launch_args+[OpaqueFunction(function=launch_setup)])

    return ld