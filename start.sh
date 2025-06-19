#!/bin/bash

# cd /home/amit-singh/Downloads/qudacopter/jMAVSim/PX4-Autopilot
# make px4_sitl gz_x500_lidar_2d_home > start_sim_script.log &

# sleep 5

# Source the ROS 2 setup file
source /opt/ros/jazzy/setup.bash

# Go to required path
cd /home/amit-singh/Downloads/qudacopter/autonomous-qudacopter


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

cd ros2_ws

ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &

sleep 1

# For parallel processing
# export MAKEFLAGS=-j2
# echo $MAKEFLAGS

# colcon build --packages-select custom_msgs --symlink-install --parallel-workers 2
# colcon build --packages-select rf2o_laser_odometry --symlink-install --parallel-workers 2
# colcon build --packages-select offboard traverse_coordinates --symlink-install --parallel-workers 2
# colcon build --packages-select odometry_transformer
# source install/local_setup.bash

# ros2 launch odometry_transformer odometry_transform.launch.py

# ros2 launch sllidar_ros2 sllidar_c1_launch.py

# ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Second ROS 2 Python command
ros2 launch traverse_coordinates coordinates_publisher.launch.py > start_script.log &

sleep 2

ros2 launch odometry_transformer odometry_transform.launch.py >> start_script.log &

sleep 2

ros2 launch slam_toolbox online_async_launch.py >> start_script.log &

# ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &

# sleep 2

# ros2 run rviz2 rviz2 &

sleep 2

# First ROS 2 Python command
ros2 launch offboard offboard_control_traverse.launch.py &

# Add more commands as needed
# ...

# wait