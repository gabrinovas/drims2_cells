from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            default_value='192.168.1.101',
            description='Robot IP address'
        ),
        
        DeclareLaunchArgument(
            'onrobot_ip', 
            default_value='192.168.1.1',
            description='OnRobot Compute Box IP'
        ),
        
        DeclareLaunchArgument(
            'gripper_type',
            default_value='onrobot_2fg7',
            description='Gripper type: robotiq or onrobot_2fg7'
        ),

        # Include robot control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('drims2_description'),
                    'launch/ur5e_2fg7/ur5e_2fg7_control.launch.py'
                ])
            ]),
            launch_arguments={
                'fake': LaunchConfiguration('fake'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'onrobot_ip': LaunchConfiguration('onrobot_ip')
            }.items()
        ),

        # Include motion server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('drims2_description'), 
                    'launch/ur5e_2fg7/ur5e_2fg7_motion_server.launch.py'
                ])
            ]),
            launch_arguments={
                'gripper_type': LaunchConfiguration('gripper_type')
            }.items()
        ),

        # Include MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('drims2_description'),
                    'launch/ur5e_2fg7/ur5e_2fg7_moveit.launch.py'
                ])
            ])
        ),
    ])