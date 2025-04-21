from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    package_dir = get_package_share_directory('offboard')

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 -p 8888 -v '
        ]],
        shell=True
    )

    offboard_control_node = Node(
        package='offboard',
        namespace='offboard',
        executable='offboard_control',
        name='control',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        micro_ros_agent,
        offboard_control_node
    ])