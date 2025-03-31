from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        # AprilTag Detection Node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/camera/image_raw'),  # Input image topic
                ('camera_info', '/camera/camera_info')  # Camera info topic
            ],
            parameters=[{
                'image_transport': 'compressed',
                'parameter_file': 'install/camera_control/share/camera_control/resource/apriltag_settings.yaml'
            }]
        ),

         # Start rqt image view
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen'
        ),
    ])
