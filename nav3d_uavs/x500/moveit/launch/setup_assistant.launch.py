from moveit_configs_utils.launches import generate_setup_assistant_launch
from nav3d_uavs.moveit import load_x500_moveit_config


def generate_launch_description():
    moveit_config = load_x500_moveit_config()
    return generate_setup_assistant_launch(moveit_config)
