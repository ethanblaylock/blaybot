#! /bin/bash

tmux new-session -d -s build

tmux set -g mouse on

tmux split-window -h

tmux send-keys -t build:0.0 'cd ~/blaybot/robot_ws && colcon build' C-m

tmux send-keys -t build:0.1 "ssh 192.168.0.120 'cd ~/blaybot/robot_ws && stdbuf -oL colcon build'" C-m

tmux attach -t build
