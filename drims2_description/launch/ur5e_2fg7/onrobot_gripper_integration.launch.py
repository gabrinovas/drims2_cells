from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'fake',
            default_value='true',
            description='Use fake hardware'
        ),
        
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.1',
            description='Robot IP address'
        ),
        
        # Include the main control launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('drims2_description'),
                    'launch/ur5e_2fg7/ur5e_2fg7_control.launch.py'
                ])
            ]),
            launch_arguments={
                'fake': LaunchConfiguration('fake'),
                'robot_ip': LaunchConfiguration('robot_ip')
            }.items()
        ),
    ])