import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('traverse_coordinates'), 'config','params.yaml')

    return LaunchDescription([
        Node(
            package='traverse_coordinates',
            executable='coordinates_publisher',
            parameters=[params_file],
            output='screen',
            shell=True,
        )
    ])
