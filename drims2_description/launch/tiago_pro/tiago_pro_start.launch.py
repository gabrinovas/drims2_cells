from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
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
    calib_args =[]
    calib_args.append(DeclareLaunchArgument(name="frame_id",              default_value="base_footprint",  description="Base link of the robot respect to the checkerboard is referred"))
    calib_args.append(DeclareLaunchArgument(name="checkerboard_frame_id", default_value="checkerboard", description="Checkerboard frame id name"))
    calib_args.append(DeclareLaunchArgument(name="checkerboar_x",         default_value="0.639",     description="Checkerboar x"))
    calib_args.append(DeclareLaunchArgument(name="checkerboar_y",         default_value="-0.102",      description="Checkerboar y"))
    calib_args.append(DeclareLaunchArgument(name="checkerboar_z",         default_value="0.846",      description="Checkerboar z"))
    calib_args.append(DeclareLaunchArgument(name="checkerboard_qx",       default_value="0.999969",  description="Checkerboar qx"))
    calib_args.append(DeclareLaunchArgument(name="checkerboard_qy",       default_value="-0.0078714",  description="Checkerboar qy"))
    calib_args.append(DeclareLaunchArgument(name="checkerboard_qz",       default_value="0.0",        description="Checkerboar qz"))
    calib_args.append(DeclareLaunchArgument(name="checkerboard_qw",       default_value="0.0",        description="Checkerboar qw"))  

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

    # tiago_pro_gripper_controller_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "tiago_pro", "tiago_pro_gripper_controller.launch.py"])
    # tiago_pro_gripper_controller_launch = IncludeLaunchDescription(
    #     launch_description_source = PythonLaunchDescriptionSource(tiago_pro_gripper_controller_path),
    #     launch_arguments={'use_sim_time': fake}.items()
    # )

    tiago_pro_rviz_path = PathJoinSubstitution([FindPackageShare("tiago_pro_moveit_config"), "launch", "moveit_rviz.launch.py"])
    # Simulation variant
    tiago_pro_rviz_sim_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(tiago_pro_rviz_path),
        launch_arguments={'use_sim_time': 'True'}.items(),
        condition=IfCondition(fake)
    )
    # Real robot variant
    tiago_pro_rviz_real_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(tiago_pro_rviz_path),
        launch_arguments={'use_sim_time': 'False'}.items(),
        condition=UnlessCondition(fake)
    )

    #calibration launch
    camera_calibration_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "tiago_pro", "tiago_pro_camera_calibration.launch.py"])
    camera_calibration_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(camera_calibration_path),
        launch_arguments = [('frame_id',       LaunchConfiguration("frame_id")),
                            ('child_frame_id', LaunchConfiguration("checkerboard_frame_id")),
                            ('x',              LaunchConfiguration("checkerboar_x")),
                            ('y',              LaunchConfiguration("checkerboar_y")),
                            ('z',              LaunchConfiguration("checkerboar_z")),
                            ('qx',             LaunchConfiguration("checkerboard_qx")),
                            ('qy',             LaunchConfiguration("checkerboard_qy")),
                            ('qz',             LaunchConfiguration("checkerboard_qz")),
                            ('qw',             LaunchConfiguration("checkerboard_qw"))] 
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

    # Set param to move_group ompl longest_valid_segment_fraction node
    gripper_node = Node(
        package='tiago_pro_setup_utils',
        executable='tiago_gripper_node',
        name='tiago_gripper_node',
        output='screen',
        parameters=[]
    )

    # Launch description including conditional Tiago launch and the table scene node
    return LaunchDescription([
        fake_arg,
        *calib_args,
        tiago_launch,
        table_scene_node,
        static_tip_frame_publisher_node,
        set_param_once_node,
        tiago_pro_rviz_sim_launch,
        tiago_pro_rviz_real_launch,
        camera_calibration_launch,
        delayed_control_server,
        gripper_node
        # tiago_pro_gripper_controller_launch,
    ])
