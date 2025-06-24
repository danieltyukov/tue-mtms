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
            Node(package="keyboard", executable="keyboard", name="keyboard_node"),
            Node(
                package="keyboard",
                executable="keyboard_to_joy.py",
                name="keyboard_to_joy_node",
                parameters=[{"config_file_name": config_file_path}],
            ),
        ]
    )
