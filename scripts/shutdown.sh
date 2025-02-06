#!/bin/bash

# Shutdown the robot
echo "Shutting down the robot"
ssh robot@192.168.0.120 "sudo shutdown -h now"

sleep 5

echo "Robot is shut down. It is safe to turn off the power."