import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    movement_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("launch_mtms"), "launch"),
                "/movement_manager_launch.py",
            ]
        )
    )

    keyboard_to_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("launch_mtms"), "launch"),
                "/keyboard_to_joy_launch.py",
            ]
        )
    )

    micro_ros_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("launch_mtms"), "launch"),
                "/microros_agent_launch.py"
            ]
        )
    )


    return LaunchDescription(
        [
            movement_manager,
            keyboard_to_joy,
            # micro_ros_agent,
        ]
    )
