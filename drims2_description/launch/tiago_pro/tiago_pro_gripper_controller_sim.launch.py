# Note: I kept this version, the cleanest one, as a backup because it is OK for simulation 
# but not compatible with the real robot because the param file is not present locally.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess

def generate_launch_description():
    drims2_description_pkg_dir = get_package_share_directory('drims2_description')

    config_arg = DeclareLaunchArgument(
        name='config',
        description='Full path to the controller configuration file',
        default_value=os.path.join(drims2_description_pkg_dir, 'config/tiago_pro/gripper_controller.yaml'),
    )

    config = LaunchConfiguration('config')

    # Spawner node
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--param-file", config,
            "--inactive"
        ],
        output="screen"
    )

    # Delay by 5 seconds
    delayed_gripper_spawner = TimerAction(
        period=5.0,
        actions=[gripper_spawner]
    )

    # Switch controllers after another delay (e.g., after 10s total)
    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'gripper_right_controller',
            '--activate', 'gripper_action_controller',
            '--strict'
        ],
        output='screen'
    )

    delayed_switch = TimerAction(
        period=12.0,  # 5s to spawn + 5s wait before switching
        actions=[switch_controllers]
    )
    return LaunchDescription([
        config_arg,
        delayed_gripper_spawner,
        delayed_switch
    ])