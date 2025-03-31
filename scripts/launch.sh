#!/bin/bash

# Kill any existing tmux session named 'launch'
tmux kill-session -t launch 2>/dev/null
sleep 0.01

# Start a new tmux session named 'launch'
tmux new-session -d -s launch

# Enable mouse support in tmux
tmux set -g mouse on

# Split the tmux session into two horizontal panes
tmux split-window -h

# Function to cleanup processes on exit
cleanup() {
    echo "Stopping ROS2 processes..."
    tmux send-keys -t launch:0.0 C-c  # Stop local ROS2 launch
    ssh robot@192.168.0.120 'pkill -f arduino_serial_node'  # Stop remote node
    exit 0
}

# Catch Ctrl+C and run cleanup function
trap cleanup SIGINT

# Launch the ROS2 launch file locally in the first pane
echo "Launching local"
tmux send-keys -t launch:0.0 'cd ~/blaybot/robot_ws && source install/setup.bash && ros2 launch start base_launch.py' C-m

sleep 5

# SSH into the robot in the second pane and start the arduino_serial_node
echo "SSH into the robot and starting robot"
tmux send-keys -t launch:0.1 "ssh -t robot@192.168.0.120 'source /opt/ros/humble/setup.bash && cd ~/blaybot/robot_ws && source install/setup.bash && ros2 launch start robot_launch.py'" C-m

# Attach to the tmux session
tmux attach -t launch

