import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_drone_core'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='ros2_drone_core',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
            parameters=[config]
        )
    ])
