from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('offboard'), 'config','params.yaml')
    # parameters = load_yaml(config_file_path)
    # print("parameters - ",parameters["ros__parameters"]["traverse_coordinates"])

    # micro_ros_agent = ExecuteProcess(
    #     cmd=[[
    #         'MicroXRCEAgent udp4 -p 8888 -v '
    #     ]],
    #     shell=True
    # )

    # micro_ros_agent = Node(
    #     package='offboard',
    #     executable='microagent_node',
    #     parameters=[params_file],
    #     output='screen',
    #     shell=True
    # )

    offboard_control_node = Node(
        package='offboard',
        executable='offboard_control_traverse',
        parameters=[params_file],
        output='screen',
        shell=True,
    )

    # package_name = 'traverse_coordinates'
    # launch_file_name = 'coordinates_publisher.launch.py' # or .xml or .yaml
    # launch_file_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'launch',
    #     launch_file_name
    # )

    # included_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_file_path),
    #     # launch_arguments={'arg_name': 'arg_value'}.items(), # optional arguments
    # )

    # traverse_coordinates_node = Node(
    #     package='offboard',
    #     executable='offboard_control_traverse',
    #     parameters=[params_file],
    #     output='screen',
    #     shell=True,
    # )

    return LaunchDescription([
        # micro_ros_agent,
        # included_launch,
        offboard_control_node
    ])