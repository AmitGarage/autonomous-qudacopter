from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os
import yaml

def generate_launch_description():
    package_dir = get_package_share_directory('offboard')
    config_file_path = os.path.join(package_dir,'config','params.yaml')
    print("config_file_path - ",config_file_path)
    parameters = load_yaml(config_file_path)
    print("parameters - ",parameters)

    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 -p 8888 -v '
        ]],
        shell=True
    )

    offboard_control_node = Node(
        package='offboard',
        namespace='offboard',
        executable='offboard_control_obstacle',
        name='control',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        micro_ros_agent,
        offboard_control_node
    ])

def load_yaml(yaml_file_path) :
    try:
        with open(yaml_file_path, 'r') as file:
            parameters = yaml.safe_load(file)
            return parameters
    except FileNotFoundError:
        print(f"Error: File not found: {yaml_file_path}")
        return None