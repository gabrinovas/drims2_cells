# Copyright 2024 National Research Council STIIMA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_configs(use_fake_hardware_str: str):
    srdf_path = os.path.join(get_package_share_directory('drims2_yumi_moveit_config'), 'config', 'platform.srdf')
    joint_limits_path = os.path.join(get_package_share_directory('drims2_yumi_moveit_config'), 'config', 'joint_limits.yaml')
    moveit_controllers_path = os.path.join(get_package_share_directory('drims2_yumi_moveit_config'), 'config', 'moveit_controllers.yaml')
    pilz_limits_path = os.path.join(get_package_share_directory('drims2_yumi_moveit_config'), 'config', 'pilz_cartesian_limits.yaml')
    robot_description_path = os.path.join(get_package_share_directory('drims2_description'), 'urdf', 'yumi', 'yumi_cell.urdf.xacro') 

    robot_description_args = {
        "use_fake_hardware": use_fake_hardware_str
    }

    moveit_config = (
        MoveItConfigsBuilder('manipulator', package_name='drims2_yumi_moveit_config')
        .robot_description(file_path=robot_description_path, mappings=robot_description_args)
        .robot_description_semantic(file_path=srdf_path)
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True)
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
        .pilz_cartesian_limits(file_path=pilz_limits_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=moveit_controllers_path)
        .robot_description_kinematics()
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("drims2_description"),
                "config/yumi/moveit_config.yaml"
            ))
        .to_moveit_configs()
    )

    return moveit_config.to_dict()


def launch_setup(context, *args, **kwargs):
    use_fake_hardware_str = LaunchConfiguration('use_fake_hardware').perform(context)
    motion_server_config_path = LaunchConfiguration('motion_server_config_path').perform(context)

    motion_server_node = Node(
        package='drims2_motion_server',
        executable='motion_server',
        name='motion_server_node',
        output='screen',
        parameters=[
            motion_server_config_path,
            get_moveit_configs(use_fake_hardware_str)
        ]
    )

    return [motion_server_node]


def generate_launch_description():
    drims2_description_pkg_dir = get_package_share_directory('drims2_description')

    motion_server_config_arg = DeclareLaunchArgument(
        'motion_server_config_path',
        default_value=os.path.join(drims2_description_pkg_dir, 'config/yumi/motion_server_config.yaml'),
        description='Full path to the config file'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='True',
        description='Whether to use fake hardware or not'
    )

    return LaunchDescription([
        motion_server_config_arg,
        use_fake_hardware_arg,
        OpaqueFunction(function=launch_setup)
    ])
