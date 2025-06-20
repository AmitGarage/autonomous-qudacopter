from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('odometry_transformer'),'params.yaml')
    # print("params_file - ",params_file)
    return LaunchDescription([
        # Node(
        #     package='odometry_transformer',
        #     executable='odometry_transformer',
        #     name='odometry_transformer',
        #     output='screen',
        # ),

        # Node(
        #     package='odometry_transformer',
        #     executable='px4_to_pose',
        #     name='px4_to_pose',
        #     output='screen',
        # ),

        Node(
            package='odometry_transformer',
            executable='px4_tf_broadcaster',
            name='px4_scan_tf_broadcaster',
            parameters=[params_file],
            output='screen',
        )
    ])
