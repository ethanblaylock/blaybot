#! /bin/bash

# Funtion to stop the ROS2 node on the robot when the script is interrupted
cleanup() {
    echo "Stopping local ROS2 nodes"
    ps aux | grep drive_node | grep -v grep | awk '{print $2}' | xargs kill -9
    ps aux | grep joy_node | grep -v grep | awk '{print $2}' | xargs kill -9
    ps aux | grep joystick_node | grep -v grep | awk '{print $2}' | xargs kill -9
    echo "Stopping the ROS2 node on the robot"
    ssh robot@192.168.0.120 "ps aux | grep arduino_serial_node | grep -v grep | awk '{print \$2}' | xargs kill -9"
}

# Trap Ctrl-C and call cleanup()
trap cleanup SIGINT

# Launch the ROS2 launch file locally
echo "Launching mobility ROS2 launch file"
cd ~/blaybot/robot_ws 
source install/setup.bash 
ros2 launch mobility mobility_launch.py &

sleep 5

# SSH into the robot and start the arduino_serial_node
echo "SSH into the robot and starting arduino_serial_node"
ssh robot@192.168.0.120 "cd ~/blaybot/robot_ws && source install/setup.bash && ros2 run mobility arduino_serial_node"