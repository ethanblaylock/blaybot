import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start arm_node
        Node(
            package='arm_control',  
            executable='arm_node',  # Adjust this to your node's executable
            name='arm_node',
            output='screen',
        ),
    ])
