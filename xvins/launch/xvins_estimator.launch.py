import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('xvins'),
        'config',
        # 'jason',  # Change to 'jason' for the Jason configuration
        'xvoyager_ros_dog.yaml'
    )

    return LaunchDescription([
        Node(
            package='xvins',
            executable='xvins_node',
            name='xvins_lifecycle_node',
            parameters=[config_path],
        )
    ])
