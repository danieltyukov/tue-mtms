from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    mirco_ros_agent = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "micro_ros_agent",
            "micro_ros_agent",
            "serial",
            "--dev",
            "/dev/ttyACM0",
        ],
        # cmd=[
        #     "evan",
        # ],
        output="screen",
    )

    return LaunchDescription([mirco_ros_agent])
