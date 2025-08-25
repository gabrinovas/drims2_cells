""" Static transform publisher parametrico """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("child_frame_id", default_value="checkerboard"),
        DeclareLaunchArgument("x", default_value="-0.673"),
        DeclareLaunchArgument("y", default_value="0.353"),
        DeclareLaunchArgument("z", default_value="0.109"),
        DeclareLaunchArgument("qx", default_value="0.0"),
        DeclareLaunchArgument("qy", default_value="0.0"),
        DeclareLaunchArgument("qz", default_value="1.0"),
        DeclareLaunchArgument("qw", default_value="0.0"),
    ]

    # Configurations
    frame_id = LaunchConfiguration("frame_id")
    child_frame_id = LaunchConfiguration("child_frame_id")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    qx = LaunchConfiguration("qx")
    qy = LaunchConfiguration("qy")
    qz = LaunchConfiguration("qz")
    qw = LaunchConfiguration("qw")

    # Node
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id,
            "--x", x,
            "--y", y,
            "--z", z,
            "--qx", qx,
            "--qy", qy,
            "--qz", qz,
            "--qw", qw,
        ],
    )

    return LaunchDescription([*declared_arguments, static_tf_node])
