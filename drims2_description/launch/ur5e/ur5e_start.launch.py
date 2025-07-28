from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
  launch_moveit_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', 'ur5e', 'ur5e_moveit.launch.py'])
  launch_moveit_and_robot_description_launch = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(launch_moveit_path),
    launch_arguments = [('fake', LaunchConfiguration("fake"))]
  )

  launch_controllers_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', 'ur5e', 'ur5e_control.launch.py'])
  launch_controllers_launch  = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(launch_controllers_path),
    launch_arguments = [('fake', LaunchConfiguration("fake"))]
  )

  motion_server_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "ur5e", "ur5e_motion_server.launch.py"])
  motion_server_launch = IncludeLaunchDescription(
      launch_description_source = PythonLaunchDescriptionSource(motion_server_path),
      launch_arguments = [('fake', LaunchConfiguration("fake"))]
  )

  delayed_control_server = TimerAction(
      period=2.0,
      actions=[motion_server_launch]
  )

  return [
      # motion_server_launch,
    launch_moveit_and_robot_description_launch,
    launch_controllers_launch,
    delayed_control_server
    # launch_controllers_launch,
    # delayed_motion_server
  ]