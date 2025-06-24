import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory("keyboard"), "config", "example_config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="movement_manager",
                executable="drive_manager",
                name="drive_manager_node",
            ),
            Node(
                package="movement_manager",
                executable="movement_manager",
                name="movement_manager_node",
            ),
            Node(
                package="movement_manager",
                executable="morph_manager",
                name="morph_manager_node",
            ),
        ]
    )
