#! /bin/bash


tmux kill-server
sleep 0.01
tmux new-session -d -t launch

tmux set -g mouse on

tmux split-window -h

tmux send-keys -t launch:0.1 "
cleanup() {
    echo 'Stopping ROS2 node on the robot'
    ssh robot@192.168.0.120 'ps aux | grep arduino_serial_node | grep -v grep | awk "{print \$2}" | xargs kill -9'
}

trap cleanup SIGINT" C-m


# Launch the ROS2 launch file locally
echo "Launching mobility ROS2 launch file"
tmux send-keys -t launch:0.0 'cd ~/blaybot/robot_ws && source install/setup.bash && ros2 launch mobility mobility_launch.py' C-m

sleep 5

# SSH into the robot and start the arduino_serial_node
echo "SSH into the robot and starting arduino_serial_node"
tmux send-keys -t launch:0.1 "ssh 192.168.0.120 'source /opt/ros/humble/setup.bash && cd ~/blaybot/robot_ws && source install/setup.bash && ros2 run mobility arduino_serial_node'" C-m

tmux attach -t launch
