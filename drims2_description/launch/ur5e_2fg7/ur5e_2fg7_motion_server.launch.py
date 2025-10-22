# Copyright 2024 National Research Council STIIMA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
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

def generate_launch_description():
    drims2_description_pkg_dir = get_package_share_directory('drims2_description')

    motion_server_config_arg = DeclareLaunchArgument(
        'motion_server_config_path',
        default_value=os.path.join(drims2_description_pkg_dir, 'config/ur5e_2fg7/motion_server_config.yaml'),
        description='Full path to the config file'
    )

    gripper_type_arg = DeclareLaunchArgument(
        'gripper_type',
        default_value='onrobot_2fg7',
        description='Gripper type: robotiq or onrobot_2fg7'
    )

    motion_server_node = Node(
        package='drims2_motion_server',
        executable='motion_server',
        name='motion_server_node',
        output='screen',
        parameters=[
            LaunchConfiguration('motion_server_config_path'),
            {'gripper_type': LaunchConfiguration('gripper_type')}
        ]
    )

    return LaunchDescription([
        motion_server_config_arg,
        gripper_type_arg,
        motion_server_node
    ])
