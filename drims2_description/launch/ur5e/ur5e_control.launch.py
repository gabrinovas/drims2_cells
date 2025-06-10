from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name='xacro')]),
          " ",
          PathJoinSubstitution([FindPackageShare("drims2_description"), "urdf", "ur5e", "ur5e_cell.urdf.xacro"]),
          " use_fake_hardware:='", LaunchConfiguration("fake").perform(context),"'"
      ]
  )
  robot_description = {'robot_description': robot_description_content}

  controllers_config = PathJoinSubstitution([FindPackageShare("drims2_description"),
   "config","ur5e","controllers.yaml"])

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

  robotiq_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_action_controller", 
               "--controller-manager", "/controller_manager"],
    output='screen',
  )

  robotiq_activation_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_activation_controller", 
               "--controller-manager", "/controller_manager"],
    output='screen',
  )

  what_to_launch = [
    controller_manager_node,
    joint_state_broadcaster_spawner,
    joint_trajectory_controller,
    robotiq_controller_spawner,
    robotiq_activation_controller_spawner,
    robot_state_publisher_node,
    ]

  return what_to_launch

def generate_launch_description():
  launch_args = []
  launch_args.append(DeclareLaunchArgument(name="fake", default_value="true", description="use fake hardware"))

  ld = LaunchDescription(launch_args+[OpaqueFunction(function=launch_setup)])
    
  return ld

  