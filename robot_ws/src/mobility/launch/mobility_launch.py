import launch
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument to specify whether to run joy node or not
        DeclareLaunchArgument('joy_node', default_value='true', description='Launch joy node'),

        # Start the joy node that publishes joystick data
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=launch.conditions.LaunchConfigurationEquals('joy_node', 'true')
        ),

        # Start drive_node
        Node(
            package='mobility',  
            executable='joystick_node',  # Adjust this to your node's executable
            name='joystick_node',
            output='screen',
        ),

        # Log message
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('joy_node', 'true'),
            msg="Joy node launched successfully"
        ),
    ])
