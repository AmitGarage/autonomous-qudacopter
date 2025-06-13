#!/bin/bash

# cd /home/amit-singh/Downloads/qudacopter/jMAVSim/PX4-Autopilot
# make px4_sitl gz_x500_lidar_2d_home &

# sleep 5

# Source the ROS 2 setup file
source /opt/ros/jazzy/setup.bash

# Go to required path
cd /home/amit-singh/Downloads/qudacopter/autonomous-qudacopter/ros2_ws

# ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan &

# For parallel processing
export MAKEFLAGS=-j2
echo $MAKEFLAGS

# colcon build --packages-select custom_msgs --symlink-install --parallel-workers 2
# colcon build --packages-select rf2o_laser_odometry --symlink-install --parallel-workers 2
# colcon build --packages-select offboard traverse_coordinates --symlink-install --parallel-workers 2
# source install/local_setup.bash

# ros2 launch sllidar_ros2 sllidar_c1_launch.py

# Second ROS 2 Python command
ros2 launch traverse_coordinates coordinates_publisher.launch.py &

sleep 1

# ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &

# sleep 2

# First ROS 2 Python command
ros2 launch offboard offboard_control_traverse.launch.py &

# Add more commands as needed
# ...

# wait