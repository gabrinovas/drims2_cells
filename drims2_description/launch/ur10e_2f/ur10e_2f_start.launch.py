from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"),
    DeclareLaunchArgument(name="robot_ip", default_value="192.168.254.100", description="Robot ip"),
    # Checkerboard calibration arguments
    DeclareLaunchArgument(name="frame_id",              default_value="base_link",    description="Base link of the robot respect to the checkerboard is referred"),
    DeclareLaunchArgument(name="checkerboard_frame_id", default_value="checkerboard", description="Checkerboard frame id name"),
    DeclareLaunchArgument(name="checkerboar_x",         default_value="-0.640",       description="Checkerboar x"),
    DeclareLaunchArgument(name="checkerboar_y",         default_value="0.321",        description="Checkerboar y"),
    DeclareLaunchArgument(name="checkerboar_z",         default_value="0.09",        description="Checkerboar z"),
    DeclareLaunchArgument(name="checkerboard_qx",       default_value="0.0039999",          description="Checkerboar qx"),
    DeclareLaunchArgument(name="checkerboard_qy",       default_value="0.999992",          description="Checkerboar qy"),
    DeclareLaunchArgument(name="checkerboard_qz",       default_value="0.0",          description="Checkerboar qz"),
    DeclareLaunchArgument(name="checkerboard_qw",       default_value="0.0",          description="Checkerboar qw"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
  launch_moveit_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', 'ur10e_2f', 'ur10e_2f_moveit.launch.py'])
  launch_moveit_and_robot_description_launch = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(launch_moveit_path),
    launch_arguments = [('fake', LaunchConfiguration("fake"))]
  )

  launch_controllers_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', 'ur10e_2f', 'ur10e_2f_control.launch.py'])
  launch_controllers_launch  = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(launch_controllers_path),
    launch_arguments = [('fake', LaunchConfiguration("fake")),
                        ('robot_ip', LaunchConfiguration("robot_ip"))]
  )

  motion_server_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "ur10e_2f", "ur10e_2f_motion_server.launch.py"])
  motion_server_launch = IncludeLaunchDescription(
      launch_description_source = PythonLaunchDescriptionSource(motion_server_path),
      launch_arguments = [('fake', LaunchConfiguration("fake"))]
  )

  camera_calibration_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "ur10e_2f", "ur10e_2f_camera_calibration.launch.py"])
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

  return [
    launch_moveit_and_robot_description_launch,
    launch_controllers_launch,
    delayed_control_server,
    camera_calibration_launch
  ]