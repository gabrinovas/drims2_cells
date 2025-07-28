from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="fake", default_value="true", description="Use fake hardware"),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    robot_description_path = PathJoinSubstitution([FindPackageShare("drims2_description"),"urdf","ur5e","ur5e_cell.urdf.xacro"]).perform(context)

    robot_description = ParameterValue(
      Command(["xacro", " ", robot_description_path, " ", "use_fake_hardware:=", LaunchConfiguration("fake")]),value_type=str
    )

    srdf_path = PathJoinSubstitution([FindPackageShare("drims2_ur5e_moveit_config"),"config","platform.srdf"]).perform(context)
    joint_limits_path = PathJoinSubstitution([FindPackageShare("drims2_ur5e_moveit_config"),"config","joint_limits.yaml"]).perform(context)
    moveit_controllers_path = PathJoinSubstitution([FindPackageShare("drims2_ur5e_moveit_config"),"config","moveit_controllers.yaml"]).perform(context)

    moveit_config = (
        MoveItConfigsBuilder("manipulator", package_name="drims2_ur5e_moveit_config")
        .robot_description_semantic(file_path=srdf_path)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=moveit_controllers_path)
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("drims2_description"),"config","ur5e","rviz_config.rviz"]).perform(context)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        output="screen"
    )

    return [
      rviz_node,
      move_group_node
    ]