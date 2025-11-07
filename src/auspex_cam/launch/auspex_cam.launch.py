from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auspex_cam',
            executable='camera_controller',
            name='camera_controller_test',
            output='screen'
        )
    ])