from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('offboard'), 'config','params.yaml')
    print("params_file_path - ",params_file)
    # parameters = load_yaml(config_file_path)
    # print("parameters - ",parameters["ros__parameters"]["traverse_coordinates"])

    # micro_ros_agent = ExecuteProcess(
    #     cmd=[[
    #         'MicroXRCEAgent udp4 -p 8888 -v '
    #     ]],
    #     shell=True
    # )

    micro_ros_agent = Node(
        package='offboard',
        executable='microagent_node',
        parameters=[params_file],
        output='screen',
        shell=True
    )

    offboard_control_node = Node(
        package='offboard',
        executable='offboard_control_traverse',
        parameters=[params_file],
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        micro_ros_agent,
        offboard_control_node
    ])

# def load_yaml(yaml_file_path) :
#     try:
#         with open(yaml_file_path, 'r') as file:
#             parameters = yaml.safe_load(file)
#             return parameters
#     except FileNotFoundError:
#         print(f"Error: File not found: {yaml_file_path}")
#         return None