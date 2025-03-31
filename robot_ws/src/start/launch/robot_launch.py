import launch
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument to specify whether to run joy node or not
        DeclareLaunchArgument('joy_node', default_value='true', description='Launch joy node'),

        # Start the camera_node
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
        ),

        # Start arduino node
        Node(
            package='mobility',  
            executable='arduino_serial_node',  # Adjust this to your node's executable
            name='arduino_serial_node',
            output='screen',
        ),

    
    ])
