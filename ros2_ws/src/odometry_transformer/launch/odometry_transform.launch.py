from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            name='px4_tf_broadcaster',
            output='screen',
        )
    ])
