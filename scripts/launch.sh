#! /bin/bash

# Launch the ROS2 launch file locally
echo "Launching mobility ROS2 launch file"
ros2 launch mobility mobility_launch.py

echo "SSH into the robot and starting arduino_serial_node"
ssh robot@192.168.0.120 "cd ~/blaybot/robot_ws && source install/setup.bash && ros2 run arduino_serial_node arduino_serial_node"