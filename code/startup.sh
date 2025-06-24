#!/usr/bin/bash

source /home/mtms/micro_ros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &

source /home/mtms/Github/MTMS/code/ros_ws/install/setup.bash
# # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &
# python3 /home/mtms/Github/MTMS/code/due_reset.py &
ros2 launch launch_mtms launch_mtms_launch.py
