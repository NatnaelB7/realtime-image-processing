from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orange',
            executable='webcam_driver',
            name='webcam_driver',
            output='screen'
        ),
        Node(
            package='red',
            executable='color_filter',
            name='color_filter',
            output='screen'
        )
    ])

