from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('drims2_description'),
        'urdf', 'ur5e_2fg7', 'ur5e_2fg7_cell.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_file])

    initial_positions_path = os.path.join(
        get_package_share_directory('drims2_description'),
        'config', 'ur5e_2fg7', 'initial_positions.yaml'
    )

    with open(initial_positions_path, 'r') as f:
        flat_positions = yaml.safe_load(f)

    joint_state_params = {
        'zeros': flat_positions['initial_positions']
    }

    print("[DEBUG] Initial joints position:")
    print(yaml.dump(joint_state_params, default_flow_style=False))

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[joint_state_params,
                        {'publish_default_positions': True}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '--display-config',
                PathJoinSubstitution([
                    FindPackageShare('drims2_description'),
                    'config', 'ur5e_2fg7', 'rviz_config.rviz'
                ])
            ]
        )
    ])
