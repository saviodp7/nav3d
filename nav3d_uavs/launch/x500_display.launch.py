# Copyright 2026 Salvatore Del Peschio
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav3d_uavs')
    xacro_file = os.path.join(pkg_share, 'x500', 'description', 'urdf', 'x500.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'x500', 'description', 'rviz', 'x500.rviz')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
            ),
        ]
    )
