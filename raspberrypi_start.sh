#!/bin/bash

# Source the ROS 2 setup file
source /opt/ros/jazzy/setup.bash

# Go to required path
cd /home/amit-singh/autonomous-qudacopter


# JSON file path
JSON_FILE="start_time_offboard_control.json"

# Check if file exists, else create it with an empty JSON object
if [ ! -f "$JSON_FILE" ]; then
    echo "{}" > "$JSON_FILE"
fi

# New content to add
NEW_CONTENT='[
    [-6,8,-5]
]'

# Write content to JSON file
echo "$NEW_CONTENT" > "$JSON_FILE"

echo "JSON file updated successfully!"


python utils/update_yaml.py config/raspberrypi_central.yaml

sleep 1

cd ros2_ws

ros2 launch sllidar_ros2 sllidar_c1_launch.py &

sleep 1

# For parallel processing
# export MAKEFLAGS=-j2
# echo $MAKEFLAGS

# colcon build --packages-select custom_msgs --symlink-install --parallel-workers 2
# colcon build --packages-select px4_msgs --symlink-install --parallel-workers 2
# colcon build --packages-select px4_ros_com --symlink-install --parallel-workers 2
# colcon build --packages-select sllidar_ros2 --symlink-install --parallel-workers 2
# colcon build --packages-select slam_toolbox --symlink-install --parallel-workers 2
# colcon build --packages-select offboard traverse_coordinates --symlink-install --parallel-workers 2
# colcon build --packages-select odometry_transformer --symlink-install --parallel-workers 2
# source install/local_setup.bash

# ros2 launch odometry_transformer odometry_transform.launch.py

# ros2 launch sllidar_ros2 sllidar_c1_launch.py

# ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Second ROS 2 Python command
ros2 launch traverse_coordinates coordinates_publisher.launch.py > ../log/start_script.log &

sleep 2

ros2 launch odometry_transformer odometry_transform.launch.py >> ../log/start_script.log &

sleep 2

ros2 launch slam_toolbox online_async_launch.py >> ../log/start_script.log &

# ros2 run rviz2 rviz2 &

sleep 2

ros2 launch offboard offboard_control_traverse.launch.py &