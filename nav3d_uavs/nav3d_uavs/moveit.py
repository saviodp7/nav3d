# Copyright 2026 Salvatore Del Peschio
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Load per-UAV MoveIt configuration from the nav3d_uavs package."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_param_builder import load_xacro, load_yaml
from moveit_configs_utils import MoveItConfigs


def load_x500_moveit_config():
    """Return the X500 configuration while keeping all files under x500/moveit."""
    package_share = Path(get_package_share_directory('nav3d_uavs'))
    moveit_dir = package_share / 'x500' / 'moveit'
    config_dir = moveit_dir / 'config'

    config = MoveItConfigs(package_path=moveit_dir)
    config.robot_description = {'robot_description': load_xacro(config_dir / 'x500.urdf.xacro')}
    config.robot_description_semantic = {
        'robot_description_semantic': load_xacro(config_dir / 'x500.srdf')
    }
    config.robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(config_dir / 'kinematics.yaml')
    }
    config.joint_limits = {
        'robot_description_planning': load_yaml(config_dir / 'joint_limits.yaml')
    }
    config.planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': False,
        'publish_robot_description_semantic': True,
    }
    config.trajectory_execution = {'allow_trajectory_execution': False}
    config.sensors_3d = load_yaml(config_dir / 'sensors_3d.yaml')
    config.planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': load_yaml(config_dir / 'ompl_planning.yaml'),
    }
    return config
