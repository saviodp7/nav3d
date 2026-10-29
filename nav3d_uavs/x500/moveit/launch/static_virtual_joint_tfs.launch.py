# Copyright 2026 Salvatore Del Peschio
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from nav3d_uavs.moveit import load_x500_moveit_config


def generate_launch_description():
    moveit_config = load_x500_moveit_config()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
