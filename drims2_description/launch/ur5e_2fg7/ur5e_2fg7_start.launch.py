from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import socket

def check_ip_connectivity(ip, port=30004, timeout=2):
    """Check if robot is reachable at given IP"""
    try:
        socket.create_connection((ip, port), timeout=timeout)
        return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False

def detect_robot_ip():
    """Try multiple IPs and return the first working one"""
    possible_ips = ["192.168.1.101", "192.168.1.105"]
    
    for ip in possible_ips:
        print(f"Testing connection to {ip}...")
        if check_ip_connectivity(ip):
            print(f"✓ Robot found at {ip}")
            return ip
    
    print("✗ No robots found at specified IPs")
    return possible_ips[0]  # Return first as fallback

def generate_launch_description():
    # Detect robot IP automatically
    detected_ip = detect_robot_ip()
    
    launch_args = [
        DeclareLaunchArgument(name="fake", default_value="false", description="use fake hardware"),
        DeclareLaunchArgument(name="robot_ip", default_value=detected_ip, description="Robot ip"),
    ]
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
    robot_ip = LaunchConfiguration("robot_ip")
    fake_conf = LaunchConfiguration("fake")
    
    launch_moveit_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', 'ur5e_2fg7', 'ur5e_2fg7_moveit.launch.py'])
    launch_moveit_and_robot_description_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(launch_moveit_path),
        launch_arguments = [('fake', fake_conf)]
    )

    launch_controllers_path = PathJoinSubstitution([FindPackageShare('drims2_description'), 'launch', 'ur5e_2fg7', 'ur5e_2fg7_control.launch.py'])
    launch_controllers_launch  = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(launch_controllers_path),
        launch_arguments = [('fake', fake_conf),
                          ('robot_ip', robot_ip)]
    )

    motion_server_path = PathJoinSubstitution([FindPackageShare("drims2_description"), "launch", "ur5e_2fg7", "ur5e_2fg7_motion_server.launch.py"])
    motion_server_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(motion_server_path),
        launch_arguments = [('fake', fake_conf)]
    )

    # FIX: Add proper delay for motion server to wait for MoveIt
    delayed_motion_server = TimerAction(
        period=15.0,  # Wait longer for MoveIt to be fully ready
        actions=[motion_server_launch]
    )

    return [
        launch_moveit_and_robot_description_launch,
        launch_controllers_launch,
        delayed_motion_server  # Use the delayed version
    ]