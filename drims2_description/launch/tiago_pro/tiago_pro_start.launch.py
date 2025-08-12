from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess

def generate_launch_description():
    # Declare the 'fake' launch argument (default is false)
    fake_arg = DeclareLaunchArgument(
        'fake',
        default_value='true',
        description='If true, launches the Tiago Gazebo simulation'
    )

    fake = LaunchConfiguration('fake')

    # Include the tiago_pro_gazebo launch file only if fake == true
    tiago_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tiago_pro_gazebo'),
                'launch',
                'tiago_pro_gazebo.launch.py'
            )
        ),
        launch_arguments={'is_public_sim': 'True',
                          'world_name': 'empty',
                          'tuck_arm': 'True'}.items(),
        condition=IfCondition(fake)
    )

    # Load the parameter YAML file for the table_scene_publisher_node
    param_file = os.path.join(
        get_package_share_directory('drims2_description'),
        'config',
        'tiago_pro',
        'tiago_utils_config.yaml'
    )

    # Define the table_scene_publisher_node with parameters
    table_scene_node = Node(
        package='tiago_pro_setup_utils',
        executable='table_scene_publisher_node',
        name='table_scene_publisher_node',
        output='screen',
        parameters=[param_file]
    )

    # Tf tip frame publisher node
    static_tip_frame_publisher_node = Node(
        package='tiago_pro_setup_utils',
        executable='static_tip_frame_publisher_node',
        name='static_tip_frame_publisher_node',
        output='screen',
        parameters=[param_file]
    )

    # Set param to move_group ompl longest_valid_segment_fraction node
    set_param_once_node = Node(
        package='tiago_pro_setup_utils',
        executable='set_param_once_node',
        name='set_param_once_node',
        output='screen',
        parameters=[]
    )

    motion_server_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "tiago_pro", "tiago_pro_motion_server.launch.py"])
    motion_server_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(motion_server_path),
        launch_arguments={'use_sim_time': fake}.items()
    )

    tiago_pro_gripper_controller_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "tiago_pro", "tiago_pro_gripper_controller.launch.py"])
    tiago_pro_gripper_controller_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(tiago_pro_gripper_controller_path),
        launch_arguments={'use_sim_time': fake}.items()
    )

    tiago_pro_rviz_path = PathJoinSubstitution([FindPackageShare("tiago_pro_moveit_config"), "launch", "moveit_rviz.launch.py"])
    tiago_pro_rviz_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(tiago_pro_rviz_path),
            launch_arguments={'use_sim_time': 'True'}.items()
    )

    delayed_control_server = TimerAction(
        period=2.0,
        actions=[motion_server_launch]
    )

    kill_gzclient = ExecuteProcess(
        cmd=[
            'pkill gzclient',
        ],
        output='screen'
    )

    kill_gzclient_action = TimerAction(
        period=8.0,  # 5s to spawn + 5s wait before switching
        actions=[kill_gzclient]
    )

    # Launch description including conditional Tiago launch and the table scene node
    return LaunchDescription([
        fake_arg,
        tiago_launch,
        table_scene_node,
        static_tip_frame_publisher_node,
        set_param_once_node,
        tiago_pro_rviz_launch,
        delayed_control_server,
        tiago_pro_gripper_controller_launch,
    ])
