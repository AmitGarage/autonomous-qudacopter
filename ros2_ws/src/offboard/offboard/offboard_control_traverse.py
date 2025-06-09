#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus #, ObstacleDistance
from sensor_msgs.msg import LaserScan
import datetime
import math
import numpy as np
import shutil
# import matplotlib
# matplotlib.use('QtAgg')
import json
import logging
from utils.convert_log_file import convert
from custom_msgs.msg import TraverseCoordinates
import queue


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        # Declare parameters
        self.declare_parameter('lidar_topic_name', '/scan')
        self.declare_parameter('static_log_file_name', '')
        self.declare_parameter('lidar_rotation_anticlockwise_direction',False)
        self.declare_parameter('lidar_angle_resolution_in_degree',0.5)
        self.declare_parameter('only_takeoff_and_land',False)
        # self.declare_parameter('baud_rate', 921600)
        # self.declare_parameter('udp_port', 8888)

        # Get parameters
        lidar_topic_name = self.get_parameter('lidar_topic_name').get_parameter_value().string_value
        self.log_file_name = self.get_parameter('static_log_file_name').get_parameter_value().string_value
        current_time = datetime.datetime.now()
        self.log_file_name = self.log_file_name.replace('start_time',str(current_time.year).zfill(4)+str(current_time.month).zfill(2)+str(current_time.day).zfill(2)+str(current_time.hour).zfill(2)+str(current_time.minute).zfill(2)+str(current_time.second).zfill(2))
        self.get_logger().info(f"log_file_name - {self.log_file_name}")


        # Create a loggr
        self.logger = logging.getLogger('offboard_control_takeoff_traverse_and_land_logger')
        self.logger.setLevel(logging.DEBUG)  # Set the desired logging level

        # Create a file handler
        file_handler = logging.FileHandler(self.log_file_name)
        file_handler.setLevel(logging.DEBUG)

        # Create a formatter
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)

        # Add the file handler to the logger
        self.logger.addHandler(file_handler)

        # Suppress console output
        self.logger.propagate = False # Prevent messages from propagating to root logger

        self.get_logger().info("Node Initialized.") # Use ROS2 logger for general info

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        # Creating lidar sensor subscriber
        self.lidar_2d_subscription = self.create_subscription(
            LaserScan, lidar_topic_name, self.obstacle_distance_callback, 10)
        # Creating traverse coordinates subscriber
        self.traverse_coordinates_subscriber = self.create_subscription(
            TraverseCoordinates, '/traverse_coordinates_topic', self.traverse_coordinates_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.traverse_coordinates_queue = queue.Queue()
        self.takeoff_height = 0.0
        self.forward_distance_x = 0.0
        self.forward_obstract_distance = [ "x" , 0.0, 0.0 ]
        self.continue_direction = [0.0,0.0]
        self.forward_distance_y = 0.0
        self.intermittent_distance_x = 0.0
        self.intermittent_distance_y = 0.0
        self.vehicle_step_distance = 0.0
        self.safe_distance_from_qudacopter = 0.6
        self.square_check = 0
        self.obstacle_distance = LaserScan()
        self.obstacle_found = False
        self.obstract_distance_x = 0.0
        self.obstract_distance_y = 0.0
        self.obstract_distance_z = 0.0
        # self.current_heading = 0.0
        self.z_achieved = False
        self.x_achieved = False
        self.y_achieved = True
        self.yaw_angle = 1.57079
        self.actual_angle = self.yaw_angle
        self.actual_angle_difference = 0.0
        self.previous_front_obstacle_found = 0.0
        self.x_rotate_achieved = False
        self.y_rotate_achieved = False
        self.drone_current_direction = [True,True]
        self.drone_current_direction_sign = [+1,+1]
        self.lidar_direction_reverse = self.get_parameter('lidar_rotation_anticlockwise_direction').get_parameter_value().bool_value
        self.lidar_angle_resolution_in_degree = self.get_parameter('lidar_angle_resolution_in_degree').get_parameter_value().double_value

        self.only_takeoff_and_land = self.get_parameter('only_takeoff_and_land').get_parameter_value().bool_value

        self.x_data = []
        self.y_data = []
        # self.z_data = []
        self.drone_x_data = []
        self.drone_y_data = []
        # self.drone_z_data = []
        # self.fig, self.ax = plt.subplots()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def traverse_coordinates_callback(self, traverse_coordinates_msg):
        # self.traverse_coordinates_queue
        self.get_logger().info(f"traverse_coordinates_msg - {traverse_coordinates_msg}")
        for row_no in range(traverse_coordinates_msg.rows) :
            self.traverse_coordinates_queue.put(traverse_coordinates_msg.data[(row_no*traverse_coordinates_msg.cols):((row_no*traverse_coordinates_msg.cols)+traverse_coordinates_msg.cols)])
        self.reset_file(traverse_coordinates_msg.file_name)

    def reset_file(self, json_file_name):
        """Overwrites the file with an empty matrix."""
        try:
            # Write an empty matrix (0x0)
            with open(json_file_name, 'w') as f:
                json.dump([], f)

            self.get_logger().info("Matrix file reset to 0x0")
        except Exception as e:
            self.get_logger().error(f"Failed to reset file: {e}")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        previous_x = self.vehicle_local_position.x
        previous_y = self.vehicle_local_position.y
        self.vehicle_local_position = vehicle_local_position
        self.drone_x_data.append(-1*self.vehicle_local_position.x)
        self.drone_y_data.append(self.vehicle_local_position.y)
        if self.forward_distance_x - self.vehicle_local_position.x >= 0:
            self.drone_current_direction[0] = True
            self.drone_current_direction_sign[0] = +1
        else:
            self.drone_current_direction[0] = False
            self.drone_current_direction_sign[0] = -1
        if self.forward_distance_y - self.vehicle_local_position.y >= 0:
            self.drone_current_direction[1] = True
            self.drone_current_direction_sign[1] = +1
        else:
            self.drone_current_direction[1] = False
            self.drone_current_direction_sign[1] = -1
        # self.drone_z_data.append(-1*self.vehicle_local_position.z)
        # self.get_logger().info(f'vehicle_local_position : {self.vehicle_local_position}') 

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        # self.get_logger().info(f'vehicle_status : {self.vehicle_status}') 

    def lidar_processing( self , lidar_msg) :

        lidar_msg = lidar_msg.ranges.tolist()
        if self.lidar_direction_reverse :
            lidar_msg = lidar_msg[::-1]

        if self.z_achieved and ( ( not self.x_achieved and self.x_rotate_achieved ) or ( not self.y_achieved and self.y_rotate_achieved ) or (self.only_takeoff_and_land)):
            obstacle_distances = [(distance_idx,lidar_msg[distance_idx]) for distance_idx in range(len(lidar_msg)) if lidar_msg[distance_idx] < np.inf]

            # self.get_logger().info(f'obstacle_distances - {obstacle_distances}')
            for obstacle_distance in obstacle_distances :
                # if math.radians(45) + math.radians(90) - self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*lidar_msg.increment) < 0 :
                #     radians = math.radians(360) + math.radians(45) + math.radians(90) - self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*lidar_msg.increment)
                # else :
                #     radians = math.radians(45) + math.radians(90) - self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*lidar_msg.increment)
                # if -math.radians(45) + self.vehicle_local_position.heading + math.radians(obstacle_distance[0]*(0.72)) < 0 :
                #     radians = math.radians(360) - math.radians(45) + self.vehicle_local_position.heading + math.radians(obstacle_distance[0]*(0.72))
                # else :
                #     radians = -math.radians(45) + self.vehicle_local_position.heading + math.radians(obstacle_distance[0]*(0.72))
                # if ( not self.x_achieved and not self.drone_current_direction[0] ) or ( not self.y_achieved and not self.drone_current_direction[1] ) :
                #     radians = math.radians(180) + (-1*math.radians(45)) + self.actual_angle_difference + self.vehicle_local_position.heading + math.radians(obstacle_distance[0]*(5))
                # else :
                #     radians = (-1*math.radians(45)) + self.actual_angle_difference + self.vehicle_local_position.heading + math.radians(obstacle_distance[0]*(5))
                # radians = (math.radians(45)) +  self.actual_angle_difference + self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*(5))
                # radians = math.radians(45) + math.radians(90) - self.actual_angle_difference - self.vehicle_local_position.heading + math.radians(obstacle_distance[0]*(0.5))
                radians = math.radians(180) - self.actual_angle_difference - self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*(self.lidar_angle_resolution_in_degree))
                # if 45 + 90 - self.yaw_angle_degree + (-1*obstacle_distance[0]*(0.72)) < 0 :
                #     radians = math.radians(360 + 45 + 90 - self.yaw_angle_degree + (-1*(obstacle_distance[0]*(0.72))))
                # else :
                #     radians = math.radians(45 + 90 - self.yaw_angle_degree + (-1*(obstacle_distance[0]*(0.72))))
                cos_value = np.cos(radians)
                sin_value = np.sin(radians)
                relative_obstacle_x = (obstacle_distance[1])*cos_value      
                relative_obstacle_y = (obstacle_distance[1])*sin_value
                absolute_obstacle_x = relative_obstacle_x + (-1*self.vehicle_local_position.x)
                absolute_obstacle_y = relative_obstacle_y + self.vehicle_local_position.y
                # absolute_obstacle_z = -1*self.vehicle_local_position.z

                self.x_data.append(absolute_obstacle_x)
                self.y_data.append(absolute_obstacle_y)
                # self.z_data.append(absolute_obstacle_z)

            # self.get_logger().info(f'lidar distance - {lidar_msg.ranges}')
            # self.get_logger().info(f'lidar angle - {[idx*lidar_msg.increment for idx in range(len(lidar_msg.ranges))]}')
            # self.get_logger().info(f'lidar angle - {[360 + 45 + 90 - self.yaw_angle_degree + (-1*(idx*lidar_msg.increment)) if 45 + 90 - self.yaw_angle_degree + (-1*idx*lidar_msg.increment) < 0 else 45 + 90 - self.yaw_angle_degree + (-1*idx*lidar_msg.increment) for idx in range(len(lidar_msg.ranges))]}')
            # self.get_logger().info(f'lidar angle - {[math.degrees(6.28319 + 0.785398 + 1.5708 - self.vehicle_local_position.heading - - math.radians(idx*lidar_msg.increment)) if 0.785398 + 1.5708 - self.vehicle_local_position.heading - math.radians(idx*lidar_msg.increment) < 0 else math.degrees(0.785398 + 1.5708 - self.vehicle_local_position.heading - math.radians(idx*lidar_msg.increment)) for idx in range(len(lidar_msg.ranges))]}')
            # self.get_logger().info(f'local position - {self.vehicle_local_position.x} - {self.vehicle_local_position.y}')
            # self.get_logger().info(f'x_data - {self.x_data[-len(obstacle_distances):]}')
            # self.get_logger().info(f'y_data - {self.y_data[-len(obstacle_distances):]}')
            map_data = {
                "x" : self.x_data,
                "y" : self.y_data,
                # "z" : self.z_data,
                "drone_x" : self.drone_x_data,
                "drone_y" : self.drone_y_data,
                # "drone_z" : self.drone_z_data
            }

            shutil.copy("/home/amit-singh/Downloads/qudacopter/tmp_map_data.json", "/home/amit-singh/Downloads/qudacopter/map_data.json")
            with open("/home/amit-singh/Downloads/qudacopter/tmp_map_data.json", "w") as outfile:
                json.dump(map_data, outfile,indent=4)
            # if not self.y_achieved and self.y_rotate_achieved :
            #     self.land()
            #     exit(0)
            # self.land()
            # exit(0)
            # self.axes.scatter(vechile_position_x,vechile_position_y,c="#fd72c0",marker=".",s=5)

    # def update_plot(self):
    #     self.ax.clear()
    #     self.ax.scatter(self.x_data, self.y_data,c="#fd72c0",marker=".",s=5)
    #     # self.ax.set_xlabel("Time/Index")
    #     # self.ax.set_ylabel("Data Value")
    #     self.fig.canvas.draw()
    #     plt.pause(0.001)

    def collision_detected(self , distances):
        if any(distance <= 0.3 for distance in distances):
            self.get_logger().info(f'All Obstacle distance: {distances}') 
            self.get_logger().info(f'May be collided : Stopping !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self.land()
            convert( self.log_file_name )
            exit(0)
    
    def obstacle_distance_callback(self, msg):
        """
        This function is called when new data is received on the topic.
        """
        # self.get_logger().info(f'{msg}')
        all_angles_distance = msg.ranges.tolist()
        if self.lidar_direction_reverse :
            all_angles_distance = all_angles_distance[::-1]
        self.logger.info(f'Obstacle distance: {self.drone_current_direction_sign} - {all_angles_distance} - {self.vehicle_local_position.heading} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z} - {self.forward_obstract_distance}')  # Print the distance
        # Add your code here to process the obstacle distance data
        self.lidar_processing( msg )
        self.collision_detected( all_angles_distance )
        # self.x_data.append(self.vehicle_local_position.x)
        # self.y_data.append(self.vehicle_local_position.y)
        # self.update_plot()
        # self.logger.info(f'All Obstacle distance: {msg}') 
        back_obstacle_found,left_obstacle_found,front_obstacle_found,right_obstacle_found,left_minimum,right_minimum,back_minimum,front_minimum,right_up_corner_distances_minimum,right_down_corner_distances_minimum,left_down_corner_distances_minimum,left_up_corner_distances_minimum,front_left_obstacle_found,front_right_obstacle_found,front_left_distances_minimum,front_right_distances_minimum = self.obstacle_and_direction( msg , 3)
        back_obstacle_found_5,left_obstacle_found_5,front_obstacle_found_5,right_obstacle_found_5,_,_,_,_,_,_,_,_,_,_,_,_ = self.obstacle_and_direction( msg , 4)
        
        if ( not self.obstacle_found ) and  (( not self.x_achieved and self.forward_obstract_distance[1] == 0.0 ) or ( not self.y_achieved and self.forward_obstract_distance[2] == 0.0 )) and (( front_obstacle_found_5 >= 2 ) or (back_obstacle_found >= 2)):
            if ( front_minimum - ( self.safe_distance_from_qudacopter + (self.safe_distance_from_qudacopter/2) ) ) < ( self.safe_distance_from_qudacopter ) :
                mid_distance =  abs((front_minimum - self.safe_distance_from_qudacopter )) + 0.25
            elif ( back_minimum - ( self.safe_distance_from_qudacopter + (self.safe_distance_from_qudacopter/2) ) ) < ( self.safe_distance_from_qudacopter ) :
                mid_distance =  abs((back_minimum - self.safe_distance_from_qudacopter )) + 0.25
            # elif (front_minimum < ( self.safe_distance_from_qudacopter*2 )) or (right_minimum < ( self.safe_distance_from_qudacopter*2 )) or (back_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_minimum < ( self.safe_distance_from_qudacopter*2 )) :
            #     mid_distance = 0.25
            elif ( front_minimum - ( self.safe_distance_from_qudacopter + (self.safe_distance_from_qudacopter/2) ) ) > 0.0 :
                mid_distance = -1*((front_minimum - ( self.safe_distance_from_qudacopter + (self.safe_distance_from_qudacopter/2) )))
            else:
                mid_distance = -1

            if not self.x_achieved :

                # if self.vehicle_local_position.x > self.forward_distance_x:
                #     self.forward_obstract_distance[1] = self.vehicle_local_position.x-(mid_distance*self.drone_current_direction_sign[0])
                # elif self.vehicle_local_position.x < self.forward_distance_x:
                #     self.forward_obstract_distance[1] = self.vehicle_local_position.x+(mid_distance*self.drone_current_direction_sign[0])
                self.forward_obstract_distance[1] = self.vehicle_local_position.x-(self.drone_current_direction_sign[0]*mid_distance)
                if mid_distance < 0 and ( self.forward_obstract_distance[1] - self.forward_distance_x )*self.drone_current_direction_sign[0] > 0:
                    self.forward_obstract_distance[1] = self.forward_distance_x
                if mid_distance > 0 and ( self.forward_obstract_distance[1] - self.forward_distance_x )*self.drone_current_direction_sign[0] < 0 :
                    if ( front_minimum - (abs( self.vehicle_local_position.x - self.forward_distance_x )) ) > ( self.safe_distance_from_qudacopter ) :
                        self.forward_obstract_distance[1] = self.vehicle_local_position.x+(self.drone_current_direction_sign[0]*abs( self.vehicle_local_position.x - self.forward_distance_x))
                
                if (right_minimum < ( self.safe_distance_from_qudacopter*2 )) and (left_minimum > ( self.safe_distance_from_qudacopter*2 )) :
                    self.forward_obstract_distance[2] = self.vehicle_local_position.y+(-0.25*self.drone_current_direction_sign[0])
                elif (right_minimum > ( self.safe_distance_from_qudacopter*2 )) and (left_minimum < ( self.safe_distance_from_qudacopter*2 )) :
                    self.forward_obstract_distance[2] = self.vehicle_local_position.y+(0.25*self.drone_current_direction_sign[0])
                # elif front_right_distances_minimum >= front_left_distances_minimum :
                #     self.forward_obstract_distance[2] = self.vehicle_local_position.y+(0.25*self.drone_current_direction_sign[0])
                # elif front_left_distances_minimum > front_right_distances_minimum :
                #     self.forward_obstract_distance[2] = self.vehicle_local_position.y+(-0.25*self.drone_current_direction_sign[0])

            if not self.y_achieved :
                # self.forward_obstract_distance[1] = self.vehicle_local_position.y-1

                # if self.vehicle_local_position.y > self.forward_distance_y:
                #     self.forward_obstract_distance[2] = self.vehicle_local_position.y-(mid_distance*self.drone_current_direction_sign[1])
                # elif self.vehicle_local_position.y < self.forward_distance_y:
                #     self.forward_obstract_distance[2] = self.vehicle_local_position.y+(mid_distance*self.drone_current_direction_sign[1])
                self.forward_obstract_distance[2] = self.vehicle_local_position.y-(self.drone_current_direction_sign[1]*mid_distance)
                if mid_distance < 0 and ( self.forward_obstract_distance[2] - self.forward_distance_y )*self.drone_current_direction_sign[1] > 0:
                    self.forward_obstract_distance[2] = self.forward_distance_y
                if mid_distance > 0 and ( self.forward_obstract_distance[2] - self.forward_distance_y )*self.drone_current_direction_sign[1] < 0 :
                    if ( front_minimum - (abs( self.vehicle_local_position.y - self.forward_distance_y )) ) > ( self.safe_distance_from_qudacopter ) :
                        self.forward_obstract_distance[2] = self.vehicle_local_position.y+(self.drone_current_direction_sign[1]*abs( self.vehicle_local_position.y - self.forward_distance_y))

                if (right_minimum < ( self.safe_distance_from_qudacopter*2 )) and (left_minimum > ( self.safe_distance_from_qudacopter*2 )) :
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x+(0.25*self.drone_current_direction_sign[1])
                elif (right_minimum > ( self.safe_distance_from_qudacopter*2 )) and (left_minimum < ( self.safe_distance_from_qudacopter*2 )) :
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x+(-0.25*self.drone_current_direction_sign[1])
                # elif front_right_distances_minimum >= front_left_distances_minimum :
                #     self.forward_obstract_distance[1] = self.vehicle_local_position.x+(-0.25*self.drone_current_direction_sign[1])
                # elif front_left_distances_minimum > front_right_distances_minimum :
                #     self.forward_obstract_distance[1] = self.vehicle_local_position.x+(0.25*self.drone_current_direction_sign[1])

            self.logger.info(f'Obstacle between traversing : {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {self.forward_obstract_distance[1]} - {self.forward_obstract_distance[2]} - {mid_distance} - {self.x_achieved} - {self.y_achieved}')
        elif self.obstacle_found :
            if self.forward_obstract_distance[0] == "x" :
                self.logger.info(f'4m obstacles : {self.drone_current_direction_sign} - {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {all_angles_distance}')
                # if left_obstacle_found < 2 and (( right_obstacle_found >= 2 ) or ( right_obstacle_found < 2 and left_obstacle_found <= right_obstacle_found ) ) and front_obstacle_found >= 2:
                #     self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+1, self.vehicle_local_position.z,self.yaw_angle)
                # elif right_obstacle_found < 2 and (( left_obstacle_found >= 2 ) or ( left_obstacle_found < 2 and right_obstacle_found <= left_obstacle_found ) ) and front_obstacle_found >= 2:
                #     self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y-1, self.vehicle_local_position.z,self.yaw_angle)
                # elif front_obstacle_found < 1 :
                #     self.logger.info(f'Forward obstacle cleared')
                #     self.obstacle_found = False
                #     self.forward_obstract_distance[1] = 0.0

                # if ( not self.traverse_direction[0] ) and ( not self.traverse_direction[1] ) and ( not self.traverse_direction[2] ) and ( not self.traverse_direction[3] ) :
                #     if front_obstacle_found >= 2 :
                #         if 

                if front_obstacle_found > 0 and front_obstacle_found <= self.previous_front_obstacle_found and self.continue_direction[1] != 0 and left_minimum >= self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter and back_minimum >= self.safe_distance_from_qudacopter :
                    if (left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 )) and abs(self.continue_direction[1]) == 1 :
                        self.continue_direction[1] = self.continue_direction[1]/4
                    elif left_minimum >= self.safe_distance_from_qudacopter*2 and right_minimum >= self.safe_distance_from_qudacopter*2 and back_minimum >= self.safe_distance_from_qudacopter*2 and abs(self.continue_direction[1]) == 0.25 :
                        self.continue_direction[1] = self.continue_direction[1]*4
 
                    if right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ) and right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ) and left_minimum > ( self.safe_distance_from_qudacopter*2 ):
                        self.continue_direction[1] = -(0.25*self.drone_current_direction_sign[0])
                        self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'Right up and down corner - Moving left in {-(0.25*self.drone_current_direction_sign[0])} direction - {self.drone_current_direction} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    if left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ) and left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ) and right_minimum > ( self.safe_distance_from_qudacopter*2 ):
                        self.continue_direction[1] = (0.25*self.drone_current_direction_sign[0])
                        self.intermittent_distance_y = self.vehicle_local_position.y+self.continue_direction[1]
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'Left up and down corner - Moving right in {+(0.25*self.drone_current_direction_sign[0])} direction - {self.drone_current_direction} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif front_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.intermittent_distance_x = self.vehicle_local_position.x+(0.25*self.drone_current_direction_sign[0])
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'Down corner - Moving up in {+(0.25*self.drone_current_direction_sign[0])} direction - {self.drone_current_direction} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif back_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.intermittent_distance_x = self.vehicle_local_position.x-(0.25*self.drone_current_direction_sign[0])
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'Up corner - Moving down in {-(0.25*self.drone_current_direction_sign[0])} direction - {self.drone_current_direction} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    else :
                        if np.sign(self.drone_current_direction_sign[0]) == np.sign(self.continue_direction[1]) :
                            y_diff = ((abs(right_minimum - self.safe_distance_from_qudacopter - 0.1)))
                        elif np.sign(self.drone_current_direction_sign[0]) != np.sign(self.continue_direction[1]) :
                            y_diff = ((abs(left_minimum - self.safe_distance_from_qudacopter - 0.1)))
                        if ( y_diff > 1 ) :
                            y_diff = np.sign(y_diff)
                        self.continue_direction[1] = y_diff*np.sign(self.continue_direction[1])
                        self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'Continue in {(self.continue_direction[1])} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and front_left_obstacle_found < front_right_obstacle_found :
                    # if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                    #     self.continue_direction[1] = self.drone_current_direction_sign[0]*(-0.25)
                    # else :
                    #     self.continue_direction[1] = self.drone_current_direction_sign[0]*(-1.0)
                    y_diff = ((abs(left_minimum - self.safe_distance_from_qudacopter - 0.1)))
                    if ( y_diff > 1 ) :
                        y_diff = np.sign(y_diff)
                    self.continue_direction[1] = self.drone_current_direction_sign[0]*y_diff*(-1)
                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'using front Going left : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and front_left_obstacle_found >= front_right_obstacle_found :
                    # if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                    #     self.continue_direction[1] = self.drone_current_direction_sign[0]*(0.25)
                    # else :
                    #     self.continue_direction[1] = self.drone_current_direction_sign[0]*(1.0)
                    
                    y_diff = ((abs(right_minimum - self.safe_distance_from_qudacopter - 0.1)))
                    if ( y_diff > 1 ) :
                        y_diff = np.sign(y_diff)
                    self.continue_direction[1] = self.drone_current_direction_sign[0]*y_diff
                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Using front Going right : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found <= right_obstacle_found :
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(-0.25)
                    else :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(-1.0)
                    
                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going left : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found >= right_obstacle_found :
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(0.25)
                    else :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(1.0)

                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going right : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and left_obstacle_found == right_obstacle_found and self.continue_direction[1] != 0.0 :

                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Continue in {(self.continue_direction[1])} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif left_obstacle_found < 2 and (( right_obstacle_found >= 2 ) or ( right_obstacle_found < 2 and left_obstacle_found <= right_obstacle_found ) ) and front_obstacle_found >= 2:
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(-0.25)
                    else :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(-1.0)

                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going left : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif right_obstacle_found < 2 and (( left_obstacle_found >= 2 ) or ( left_obstacle_found < 2 and right_obstacle_found <= left_obstacle_found ) ) and front_obstacle_found >= 2:
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(0.25)
                    else :
                        self.continue_direction[1] = self.drone_current_direction_sign[0]*(1.0)

                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going right : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found <= 1 and left_minimum >= self.safe_distance_from_qudacopter*2 and right_minimum < self.safe_distance_from_qudacopter :
                    self.continue_direction[1] = self.drone_current_direction_sign[0]*(-0.25)

                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)

                    self.logger.info(f'Going left because of right obstacle : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found <= 1 and left_minimum < self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter*2 :
                    self.continue_direction[1] = self.drone_current_direction_sign[0]*(0.25)

                    self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y , self.takeoff_height,self.yaw_angle)

                    self.logger.info(f'Going right because of left obstacle : {(self.continue_direction[1])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found < 1 and left_minimum >= self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter and back_minimum >= self.safe_distance_from_qudacopter:
                    self.logger.info(f'Forward obstacle cleared and setting to 0 - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                    self.obstacle_found = False
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                    self.continue_direction[0] = 0.0
                    self.continue_direction[1] = 0.0
                    self.intermittent_distance_x = self.vehicle_local_position.x
                    self.intermittent_distance_y = self.vehicle_local_position.y
                else :
                    if self.continue_direction[1] != 0.0 :
                        self.intermittent_distance_y = self.vehicle_local_position.y+(self.continue_direction[1])
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'Continue in {(self.continue_direction[1])} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    else :
                        self.logger.info(f'Continue direction not set')

            
            elif self.forward_obstract_distance[0] == "y" :

                self.logger.info(f'y 4m obstacles : {self.drone_current_direction_sign} - {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {all_angles_distance}')
                if front_obstacle_found > 0 and front_obstacle_found <= self.previous_front_obstacle_found and self.continue_direction[0] != 0.0 and left_minimum >= self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter and back_minimum >= self.safe_distance_from_qudacopter:
                    if ( left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 )) and abs(self.continue_direction[0]) == 1 :
                        self.continue_direction[0] = self.continue_direction[0]/4
                    elif left_minimum >= self.safe_distance_from_qudacopter*2 and right_minimum >= self.safe_distance_from_qudacopter*2 and back_minimum >= self.safe_distance_from_qudacopter*2 and abs(self.continue_direction[0]) == 0.25 :
                        self.continue_direction[0] = self.continue_direction[0]*4

                    if (right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and back_minimum > ( self.safe_distance_from_qudacopter*2 ) :
                        self.intermittent_distance_y = self.vehicle_local_position.y-(self.drone_current_direction_sign[1]*0.25)
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Up corner - Moving Down in {-(self.drone_current_direction_sign[1]*0.25)} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif (right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and (left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and front_minimum > ( self.safe_distance_from_qudacopter*2 ) :
                        self.intermittent_distance_y = self.vehicle_local_position.y+(self.drone_current_direction_sign[1]*0.25)
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Down corner - Moving Up in {(self.drone_current_direction_sign[1]*0.25)} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif (right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and (right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and left_minimum > ( self.safe_distance_from_qudacopter*2 ) :
                        self.intermittent_distance_y = self.vehicle_local_position.y+(self.drone_current_direction_sign[1]*0.25)
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Right corners - Moving left in {(self.drone_current_direction_sign[1]*0.25)} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif (left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) and right_minimum > ( self.safe_distance_from_qudacopter*2 ) :
                        self.intermittent_distance_y = self.vehicle_local_position.y-(self.drone_current_direction_sign[1]*0.25)
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Left corners - Moving Right in {-(self.drone_current_direction_sign[1]*0.25)} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif front_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.intermittent_distance_y = self.vehicle_local_position.y+(self.drone_current_direction_sign[1]*0.25)
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Down corner - Moving up in {(self.drone_current_direction_sign[1]*0.25)} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif back_minimum > ( self.safe_distance_from_qudacopter*2 ) and right_minimum > ( self.safe_distance_from_qudacopter*2 ) and right_down_corner_distances_minimum > ( self.safe_distance_from_qudacopter*2 ) and (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) :
                        self.continue_direction[0] = (self.drone_current_direction_sign[1]*0.25)
                        self.intermittent_distance_x = self.vehicle_local_position.x+self.continue_direction[0]
                        self.intermittent_distance_y = self.vehicle_local_position.y-(self.drone_current_direction_sign[1]*0.25)
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Left Up corner - Moving right down in {self.continue_direction[0]} {-(self.drone_current_direction_sign[1]*0.25)} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    elif back_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.intermittent_distance_y = self.vehicle_local_position.y-(self.drone_current_direction_sign[1]*0.25)
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Up corner - Moving down in {-(self.drone_current_direction_sign[1]*0.25)} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y} - {self.vehicle_local_position.z}')
                    else :
                        if np.sign(self.drone_current_direction_sign[1]) == np.sign(self.continue_direction[0]) :
                            x_diff = ((abs(left_minimum - self.safe_distance_from_qudacopter - 0.1)))
                        elif np.sign(self.drone_current_direction_sign[1]) != np.sign(self.continue_direction[0]) :
                            x_diff = ((abs(right_minimum - self.safe_distance_from_qudacopter - 0.1)))
                        if ( x_diff > 1 ) :
                            x_diff = np.sign(x_diff)
                        self.continue_direction[0] = x_diff*np.sign(self.continue_direction[0])
                        self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'y Continue in {(self.continue_direction[0])} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found >= 2 and front_left_obstacle_found < front_right_obstacle_found :
                    # if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                    #     self.continue_direction[0] = self.drone_current_direction_sign[1]*(0.25)
                    # else :
                    #     self.continue_direction[0] = self.drone_current_direction_sign[1]*(1.0)
                    x_diff = ((abs(left_minimum - self.safe_distance_from_qudacopter - 0.1)))
                    if ( x_diff > 1 ) :
                        x_diff = np.sign(x_diff)
                    self.continue_direction[0] = self.drone_current_direction_sign[1]*x_diff
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Using front Going left : {(self.continue_direction[0])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found >= 2 and front_right_obstacle_found <= front_left_obstacle_found :
                    # if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                    #     self.continue_direction[0] = self.drone_current_direction_sign[1]*(-0.25)
                    # else :
                    #     self.continue_direction[0] = self.drone_current_direction_sign[1]*(-1.0)
                    x_diff = ((abs(right_minimum - self.safe_distance_from_qudacopter - 0.1)))
                    if ( x_diff > 1 ) :
                        x_diff = np.sign(x_diff)
                    self.continue_direction[0] = self.drone_current_direction_sign[1]*x_diff*(-1)
                    # self.continue_direction[0] = self.drone_current_direction_sign[1]*((abs(front_minimum - self.safe_distance_from_qudacopter - 0.1)))
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Using front Going right : {(self.continue_direction[0])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found <= right_obstacle_found :
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(0.25)
                    else :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(1.0)
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going left : {(self.continue_direction[0]*np.sign(self.vehicle_local_position.x))} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found >= right_obstacle_found :
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(-0.25)
                    else :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(-1.0)
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going right : {(self.continue_direction[0]*np.sign(self.vehicle_local_position.x))} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found >= 2 and left_obstacle_found == right_obstacle_found and self.continue_direction[0] != 0.0 :
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Continue in {(self.continue_direction[0])} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif left_obstacle_found < 2 and (( right_obstacle_found >= 2 ) or ( right_obstacle_found < 2 and left_obstacle_found <= right_obstacle_found ) ) and front_obstacle_found >= 2:
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(0.25)
                    else :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(1.0)
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going left : {(self.continue_direction[0])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif right_obstacle_found < 2 and (( left_obstacle_found >= 2 ) or ( left_obstacle_found < 2 and right_obstacle_found <= left_obstacle_found ) ) and front_obstacle_found >= 2:
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(-0.25)
                    else :
                        self.continue_direction[0] = self.drone_current_direction_sign[1]*(-1)
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.logger.info(f'Going right : {(self.continue_direction[0])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found <= 1 and left_minimum >= self.safe_distance_from_qudacopter*2 and right_minimum < self.safe_distance_from_qudacopter :
                    self.continue_direction[0] = self.drone_current_direction_sign[1]*(0.25)
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)

                    self.logger.info(f'Going left because of right obstacle : {(self.continue_direction[0])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found <= 1 and left_minimum < self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter*2 :
                    self.continue_direction[0] = self.drone_current_direction_sign[1]*(-0.25)
                    self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)

                    self.logger.info(f'Going right because of left obstacle : {(self.continue_direction[0])} - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                elif front_obstacle_found < 1 and left_minimum >= self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter and back_minimum >= self.safe_distance_from_qudacopter:
                    self.logger.info(f'Forward obstacle cleared and setting to 0 ')
                    self.obstacle_found = False
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                    self.continue_direction[0] = 0.0
                    self.continue_direction[1] = 0.0
                    self.intermittent_distance_x = self.vehicle_local_position.x
                    self.intermittent_distance_y = self.vehicle_local_position.y
                else :
                    if self.continue_direction[0] != 0.0 :
                        self.intermittent_distance_x = self.vehicle_local_position.x+(self.continue_direction[0])
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                        self.logger.info(f'Continue in {(self.continue_direction[0])} direction - {self.intermittent_distance_x} - {self.intermittent_distance_y}')
                    else :
                        self.logger.info(f'Continue direction not set')
        self.previous_front_obstacle_found = front_obstacle_found

    def obstacle_and_direction( self, msg, threshold ):
        msg = msg.ranges.tolist()
        if self.lidar_direction_reverse :
            msg = msg[::-1]

        ## Degree range - (-25:) and (:25)
        obstacle_distances_front = msg[-1*(int(25/self.lidar_angle_resolution_in_degree)):]+msg[:int(25/self.lidar_angle_resolution_in_degree)] #msg[-50:]+msg[:50] #msg[40:140] #msg[25:100] #msg[5:13]
        ## Degree range - ( 25+40 : 25+40+50 )
        obstacle_distances_right = msg[int(65/self.lidar_angle_resolution_in_degree):int(115/self.lidar_angle_resolution_in_degree)] #msg[130:230] #msg[220:320] #msg[150:225] #msg[23:31]
        ## Degree range - ( 25+40+50+40 : 25+40+50+40+50 )
        obstacle_distances_back = msg[int(155/self.lidar_angle_resolution_in_degree):int(205/self.lidar_angle_resolution_in_degree)] #msg[310:410] #msg[275:350] #msg[41:49]
        ## Degree range - ( 25+40+50+40+50+40 : 25+40+50+40+50+40+50 )
        obstacle_distances_left = msg[int(245/self.lidar_angle_resolution_in_degree):int(295/self.lidar_angle_resolution_in_degree)] #msg[490:590] #msg[400:475] #msg[59:67] 

        ## Degree range - ( 25+10 : 25+10+20 )
        right_up_corner_distances = msg[int(35/self.lidar_angle_resolution_in_degree):int(55/self.lidar_angle_resolution_in_degree)] #msg[70:110] #msg[-20:]+msg[:20] #msg[-20:]+msg[:20] #msg[-2:]+msg[2:]
        ## Degree range - ( 25+10+20+50+20 : 25+10+20+50+20+20 )
        right_down_corner_distances = msg[int(125/self.lidar_angle_resolution_in_degree):int(145/self.lidar_angle_resolution_in_degree)] #msg[250:290] #msg[160:200] #msg[105:145] #msg[16:20]
        ## Degree range - ( 25+10+20+50+20+20+50+20 : 25+10+20+50+20+20+50+20+20 )
        left_down_corner_distances = msg[int(215/self.lidar_angle_resolution_in_degree):int(235/self.lidar_angle_resolution_in_degree)] #msg[430:470] #msg[340:380] #msg[230:270] #msg[34:38]
        ## Degree range - ( 25+10+20+50+20+20+50+20+20+50+20 : 25+10+20+50+20+20+50+20+20+50+20+20 )
        left_up_corner_distances = msg[int(305/self.lidar_angle_resolution_in_degree):int(325/self.lidar_angle_resolution_in_degree)] #msg[610:650] #msg[520:560] #msg[355:395] #msg[52:56]

        front_left_distance = msg[-1*(int(25/self.lidar_angle_resolution_in_degree)):] #msg[-50:] #msg[40:90] #msg[25:50] #msg[5:9]
        front_right_distance = msg[:int(25/self.lidar_angle_resolution_in_degree)] #msg[:50] #msg[90:140] #msg[50:100] #msg[9:13]

        # obstacle_distances_centre = msg[61:65]
        back_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_back)])
        left_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_left)])
        front_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_front)])
        right_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_right)])

        front_left_obstacle_found = len([*filter(lambda x: x < threshold, front_left_distance)])
        front_right_obstacle_found = len([*filter(lambda x: x < threshold, front_right_distance)])

        left_minimum = min(obstacle_distances_left)
        right_minimum = min(obstacle_distances_right)
        back_minimum = min(obstacle_distances_back)
        front_minimum = min(obstacle_distances_front)
        right_up_corner_distance_minimum = min(right_up_corner_distances)
        right_down_corner_distances_minimum = min(right_down_corner_distances)
        left_down_corner_distances_minimum = min(left_down_corner_distances)
        left_up_corner_distances_minimum = min(left_up_corner_distances)
        front_left_distances_minimum = min(front_left_distance)
        front_right_distances_minimum = min(front_right_distance)
        return back_obstacle_found,left_obstacle_found,front_obstacle_found,right_obstacle_found,left_minimum,right_minimum,back_minimum,front_minimum,right_up_corner_distance_minimum,right_down_corner_distances_minimum,left_down_corner_distances_minimum,left_up_corner_distances_minimum,front_left_obstacle_found,front_right_obstacle_found,front_left_distances_minimum,front_right_distances_minimum

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self,move_type="position"):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        if move_type == "position" :
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
        elif move_type == "velocity" :
            msg.position = True
            msg.velocity = True
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
        elif move_type == "acceleration" :
            msg.position = False
            msg.velocity = False
            msg.acceleration = True
            msg.attitude = False
            msg.body_rate = False
        elif move_type == "rotate" :
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = True
            msg.body_rate = True
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, move_type: str, x: float, y: float, z: float,yaw_angle: float = 0.0, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        if move_type == "position" :
            msg.position = [x, y, z]
            msg.yaw = yaw_angle  # (90 degree)
        elif move_type == "velocity" :
            msg.position = [x, y, z]
            msg.velocity = [vx, vy, vz]
            # self.get_logger().info(f"Publishing velocity setpoints {msg}")
            msg.yaw = yaw_angle  # (90 degree)
        elif move_type == "acceleration" :
            msg.acceleration = [x, y, z]
        elif move_type == "rotate" :
            msg.position = [x, y, z]
            msg.yaw = yaw_angle
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal("position")
        # self.get_logger().info(f"self.obstacle_distance - {self.obstacle_distance}")

        if self.offboard_setpoint_counter == 11:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_setpoint_counter += 1
            if not self.traverse_coordinates_queue.empty() :
                self.forward_distance_x, self.forward_distance_y, self.takeoff_height = self.traverse_coordinates_queue.get()
                self.get_logger().info(f"Position has been set to traverse - {self.forward_distance_x} - {self.forward_distance_y} - {self.takeoff_height}")
        if not self.obstacle_found :
            if self.traverse_coordinates_queue.empty() and self.forward_distance_x == 0.0 and self.forward_distance_y == 0.0 and self.takeoff_height == 0.0:
                self.get_logger().info(f"No postions found to traverse")
                self.land()
                convert( self.log_file_name )
                exit(0)
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 0:
                if ( not self.z_achieved ) and round(self.vehicle_local_position.z,0) != self.takeoff_height :
                    self.publish_position_setpoint("position", 0.0, 0.0, self.takeoff_height,self.yaw_angle)
                elif ( not self.z_achieved ) and round(self.vehicle_local_position.z,0) == self.takeoff_height :
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.z}")
                    # self.publish_offboard_control_heartbeat_signal("rotate")
                    # self.yaw_angle -= 1.57079
                    # self.publish_position_setpoint("rotate", 0.0, 0.0, self.takeoff_height,self.yaw_angle)
                    self.z_achieved = True
                elif self.z_achieved and round(self.vehicle_local_position.heading,2) != round(self.yaw_angle,2):
                    self.get_logger().info(f"rotating - {self.vehicle_local_position.heading} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.publish_offboard_control_heartbeat_signal("rotate")
                #     self.publish_position_setpoint("rotate", 0.0, 0.0, self.takeoff_height,self.yaw_angle)
                elif self.z_achieved and round(self.vehicle_local_position.heading,2) == round(self.yaw_angle,2) :
                    self.get_logger().info(f"{self.square_check} {self.vehicle_local_position.heading} rotate completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.square_check += 1
                    # if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0  and abs(self.yaw_angle) == 1.57079:
                    #     self.yaw_angle = 0.0
                    # elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0  and abs(self.yaw_angle) == 1.57079 :
                    #     self.yaw_angle = 3.14158
                    self.get_logger().info(f"Before : {( self.forward_distance_y - self.vehicle_local_position.y )} - {self.yaw_angle}")
                    if self.actual_angle == -1.57079 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle += 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle-= 1.57079
                    elif self.actual_angle == 1.57079 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle -= 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle += 1.57079
                    elif self.actual_angle == 4.71237 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle += 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle -= 1.57079
                    elif self.actual_angle == -4.71237 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle -= 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle += 1.57079

                    if self.actual_angle > 0 and self.actual_angle > 3.14158 :
                        self.yaw_angle = self.actual_angle - 6.28316
                        self.actual_angle_difference = 6.28316
                    elif self.actual_angle < 0 and self.actual_angle < -3.14158 :
                        self.yaw_angle = 6.28316 + self.actual_angle
                        self.actual_angle_difference = -6.28316
                    else :
                        self.yaw_angle = self.actual_angle
                        self.actual_angle_difference = 0.0

                    if abs(self.yaw_angle) == 0.0 :
                        self.actual_angle_difference = 0.0
                        self.actual_angle = 0.0

                    self.get_logger().info(f"After : {( self.forward_distance_y - self.vehicle_local_position.y )} - {self.yaw_angle} - {self.actual_angle_difference} - {self.actual_angle}")
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                    self.intermittent_distance_x = self.vehicle_local_position.x
                    self.intermittent_distance_y = self.vehicle_local_position.y
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    # time.sleep(30)

            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 1 and self.only_takeoff_and_land :
                self.land()
                convert( self.log_file_name )
                exit(0)
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 1:

                if ( not self.x_achieved ) and ( not self.x_rotate_achieved ) and ( round(self.vehicle_local_position.heading,2) < round(self.yaw_angle,2)-0.005 or round(self.vehicle_local_position.heading,2) > round(self.yaw_angle,2)+0.005 ):
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.get_logger().info(f"x rotating - {self.vehicle_local_position.heading} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.publish_offboard_control_heartbeat_signal("rotate")
                #     self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif ( not self.x_achieved ) and ( not self.x_rotate_achieved ) and round(self.vehicle_local_position.heading,2) > round(self.yaw_angle,2)-0.005 and round(self.vehicle_local_position.heading,2) < round(self.yaw_angle,2)+0.005 :
                    self.get_logger().info(f"x {self.square_check} {self.vehicle_local_position.heading} rotate completed - {self.vehicle_local_position.heading} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.x_rotate_achieved = True
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                elif ( not self.x_achieved ) and self.x_rotate_achieved and self.forward_obstract_distance[1] != 0.0 and self.forward_obstract_distance[2] != 0.0 and (( round(self.vehicle_local_position.x,2) <= round(self.forward_obstract_distance[1],2) - 0.25 or round(self.vehicle_local_position.x,2) >= round(self.forward_obstract_distance[1],2) + 0.25 ) or ( round(self.vehicle_local_position.y,2) <= round(self.forward_obstract_distance[2],2) - 0.25 or round(self.vehicle_local_position.y,2) >= round(self.forward_obstract_distance[2],2) + 0.25 )):

                    x_diff = (self.forward_obstract_distance[1] - self.vehicle_local_position.x)
                    y_diff = (self.forward_obstract_distance[2] - self.vehicle_local_position.y)

                    if ( x_diff < -1 ) or ( x_diff > 1 ) :
                        x_diff = np.sign(x_diff)
                    if ( y_diff < -1 ) or ( y_diff > 1 ) :
                        y_diff = np.sign(y_diff)

                    self.intermittent_distance_x = self.vehicle_local_position.x+x_diff
                    self.intermittent_distance_y = self.vehicle_local_position.y+y_diff

                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                    self.get_logger().info(f'x Going towards mid point using both x and y - {self.intermittent_distance_x} - {self.intermittent_distance_y}')

                elif ( not self.x_achieved ) and self.x_rotate_achieved and self.forward_obstract_distance[1] != 0.0 and self.forward_obstract_distance[2] != 0.0 and round(self.vehicle_local_position.x,2) > round(self.forward_obstract_distance[1],2) - 0.25 and round(self.vehicle_local_position.x,2) < round(self.forward_obstract_distance[1],2) + 0.25 and round(self.vehicle_local_position.y,2) > round(self.forward_obstract_distance[2],2) - 0.25 and round(self.vehicle_local_position.y,2) < round(self.forward_obstract_distance[2],2) + 0.25:
                    self.get_logger().info(f'x Achieved at mid point using both x and y, obstacle ahead - {self.vehicle_local_position.x} - {self.vehicle_local_position.y}')
                    if not self.check_destination( "x" ) :
                        self.forward_obstract_distance[0] = "x"
                        self.obstacle_found = True
                elif ( not self.x_achieved ) and self.x_rotate_achieved and self.forward_obstract_distance[1] != 0.0 and self.forward_obstract_distance[2] == 0.0 and ( round(self.vehicle_local_position.x,2) <= round(self.forward_obstract_distance[1],2)-0.25  or round(self.vehicle_local_position.x,2) >= round(self.forward_obstract_distance[1],2)+0.25 ):
                    
                    x_diff = self.forward_obstract_distance[1] - self.vehicle_local_position.x

                    if ( x_diff < -1 ) or ( x_diff > 1 ) :
                        x_diff = np.sign(x_diff)

                    self.intermittent_distance_x = self.vehicle_local_position.x+x_diff
                    
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.get_logger().info(f'x Going towards mid point using x - {self.intermittent_distance_x} - {self.intermittent_distance_y}')

                elif ( not self.x_achieved ) and self.x_rotate_achieved and self.forward_obstract_distance[1] != 0.0 and self.forward_obstract_distance[2] == 0.0 and ( round(self.vehicle_local_position.x,2) > round(self.forward_obstract_distance[1],2)-0.25 and round(self.vehicle_local_position.x,2) < round(self.forward_obstract_distance[1],2)+0.25 ) :
                    self.get_logger().info(f'x Achieved at mid point using x, obstacle ahead - {self.vehicle_local_position.x} - {self.vehicle_local_position.y}')
                    if not self.check_destination( "x" ) :
                        self.forward_obstract_distance[0] = "x"
                        self.obstacle_found = True
                elif ( not self.x_achieved ) and self.x_rotate_achieved and self.forward_obstract_distance[1] == 0.0 and self.forward_obstract_distance[2] == 0.0 and ( round(self.vehicle_local_position.x,2) <= round(self.forward_distance_x,2)-0.25 or round(self.vehicle_local_position.x,2) >= round(self.forward_distance_x,2)+0.25 ):
                    
                    x_diff = self.forward_distance_x - self.vehicle_local_position.x

                    if ( x_diff < -1 ) or ( x_diff > 1 ) :
                        x_diff = np.sign(x_diff)

                    self.intermittent_distance_x = self.vehicle_local_position.x+x_diff

                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.get_logger().info(f'x Going towards required x - {self.intermittent_distance_x} - {self.intermittent_distance_y}')

                elif ( not self.x_achieved ) and self.x_rotate_achieved and self.forward_obstract_distance[1] == 0.0 and self.forward_obstract_distance[2] == 0.0 and ( round(self.vehicle_local_position.x,2) > round(self.forward_distance_x,2)-0.25 and round(self.vehicle_local_position.x,2) < round(self.forward_distance_x,2)+0.25 ) :
                    self.get_logger().info(f"x {self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.x_achieved = True
                    # self.publish_offboard_control_heartbeat_signal("rotate")
                    # self.yaw_angle += 1.57079
                #     self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                    # self.yaw_angle += 1.57079
                    # if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0 :
                    #     self.yaw_angle += 1.57079
                    # else :
                    #     self.yaw_angle -= 1.57079
                    self.get_logger().info(f"Before : {( self.forward_distance_y - self.vehicle_local_position.y )} - {self.yaw_angle}")
                    # if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0  and abs(self.yaw_angle) == 0.0:
                    #     self.yaw_angle = 1.57079
                    # elif ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0  and abs(self.yaw_angle) == 3.14158 :
                    #     self.yaw_angle = 1.57079
                    # elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0  and abs(self.yaw_angle) == 0.0 :
                    #     self.yaw_angle = -1.57079
                    # elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0  and abs(self.yaw_angle) == 3.14158 :
                    #     self.yaw_angle = -1.57079


                    if self.actual_angle == 0.0 :
                        if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0 :
                            self.actual_angle += 1.57079
                        elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0 :
                            self.actual_angle-= 1.57079
                    elif self.actual_angle == 3.14158 :
                        if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0 :
                            self.actual_angle -= 1.57079
                        elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0 :
                            self.actual_angle += 1.57079
                    elif self.actual_angle == -3.14158 :
                        if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0 :
                            self.actual_angle -= 1.57079
                        elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0 :
                            self.actual_angle += 1.57079

                    if self.actual_angle > 0 and self.actual_angle > 3.14158 :
                        self.yaw_angle = self.actual_angle - 6.28316
                        self.actual_angle_difference = 6.28316
                    elif self.actual_angle < 0 and self.actual_angle < -3.14158 :
                        self.yaw_angle = 6.28316 + self.actual_angle
                        self.actual_angle_difference = -6.28316
                    else :
                        self.yaw_angle = self.actual_angle
                        self.actual_angle_difference = 0.0

                    if abs(self.yaw_angle) == 0.0 :
                        self.actual_angle_difference = 0.0
                        self.actual_angle = 0.0

                    self.get_logger().info(f"After : {( self.forward_distance_y - self.vehicle_local_position.y )} - {self.yaw_angle} - {self.actual_angle_difference} - {self.actual_angle}")
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                    self.intermittent_distance_x = self.vehicle_local_position.x
                    self.intermittent_distance_y = self.vehicle_local_position.y
                    if ( self.vehicle_local_position.y < self.forward_distance_y-0.25 ) or ( self.vehicle_local_position.y > self.forward_distance_y+0.25 ) :
                        self.y_achieved = False
                        self.y_rotate_achieved = False
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    elif not self.traverse_coordinates_queue.empty() :
                        self.y_achieved = False
                        self.y_rotate_achieved = False
                        self.forward_distance_x, self.forward_distance_y, self.takeoff_height = self.traverse_coordinates_queue.get()
                        self.get_logger().info(f"New Position has been set to traverse - {self.forward_distance_x} - {self.forward_distance_y} - {self.takeoff_height}")
                    else :
                        self.land()
                        convert( self.log_file_name )
                        exit(0)
                
                elif ( not self.y_achieved ) and ( not self.y_rotate_achieved ) and ( round(self.vehicle_local_position.heading,2) < (round(self.yaw_angle,2)-0.005) or round(self.vehicle_local_position.heading,2) > (round(self.yaw_angle,2)+0.005) ):
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.get_logger().info(f"y rotating - {round(self.vehicle_local_position.heading,2)} - {round(self.yaw_angle,2)} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.publish_offboard_control_heartbeat_signal("rotate")
                #     self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif ( not self.y_achieved ) and ( not self.y_rotate_achieved ) and round(self.vehicle_local_position.heading,2) > (round(self.yaw_angle,2)-0.005) and round(self.vehicle_local_position.heading,2) < (round(self.yaw_angle,2)+0.005) :
                    self.get_logger().info(f"y {self.square_check} {self.vehicle_local_position.heading} rotate completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    # self.square_check += 1
                    self.y_rotate_achieved = True
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0

                elif ( not self.y_achieved ) and self.y_rotate_achieved and self.forward_obstract_distance[1] != 0.0 and self.forward_obstract_distance[2] != 0.0 and (( round(self.vehicle_local_position.x,2) <= round(self.forward_obstract_distance[1],2)-0.25 or round(self.vehicle_local_position.x,2) >= round(self.forward_obstract_distance[1],2)+0.25 ) or ( round(self.vehicle_local_position.y,2) <= round(self.forward_obstract_distance[2],2)-0.25 or round(self.vehicle_local_position.y,2) >= round(self.forward_obstract_distance[2],2)+0.25 )):
                    # self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)

                    x_diff = (self.forward_obstract_distance[1] - self.vehicle_local_position.x)
                    y_diff = (self.forward_obstract_distance[2] - self.vehicle_local_position.y)

                    if ( x_diff < -1 ) or ( x_diff > 1 ) :
                        x_diff = np.sign(x_diff)
                    if ( y_diff < -1 ) or ( y_diff > 1 ) :
                        y_diff = np.sign(y_diff)

                    self.intermittent_distance_x = self.vehicle_local_position.x+x_diff
                    self.intermittent_distance_y = self.vehicle_local_position.y+y_diff

                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.get_logger().info(f'y Going towards mid point using both x and y - {self.intermittent_distance_x} - {self.intermittent_distance_y}')

                elif ( not self.y_achieved ) and self.y_rotate_achieved and self.forward_obstract_distance[1] != 0.0 and self.forward_obstract_distance[2] != 0.0 and round(self.vehicle_local_position.x,2) > round(self.forward_obstract_distance[1],2)-0.25 and round(self.vehicle_local_position.x,2) < round(self.forward_obstract_distance[1],2)+0.25  and round(self.vehicle_local_position.y,2) > round(self.forward_obstract_distance[2],2)-0.25 and round(self.vehicle_local_position.y,2) < round(self.forward_obstract_distance[2],2)+0.25 :
                    self.get_logger().info(f'y Achieved at mid point using x and y , obstacle ahead - {self.vehicle_local_position.x} - {self.vehicle_local_position.y}')
                    if not self.check_destination( "y" ) :
                        self.forward_obstract_distance[0] = "y"
                        self.obstacle_found = True
                elif ( not self.y_achieved ) and self.y_rotate_achieved and self.forward_obstract_distance[1] == 0.0 and self.forward_obstract_distance[2] != 0.0 and ( round(self.vehicle_local_position.y,2) <= round(self.forward_obstract_distance[2],2)-0.25 or round(self.vehicle_local_position.y,2) >= round(self.forward_obstract_distance[2],2)+0.25 ) :
                    # self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)

                    y_diff = self.forward_obstract_distance[2] - self.vehicle_local_position.y

                    if ( y_diff < -1 ) or ( y_diff > 1 ) :
                        y_diff = np.sign(y_diff)

                    self.intermittent_distance_y = self.vehicle_local_position.y+y_diff

                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                    self.get_logger().info(f'y Going towards mid point using y , obstacle ahead - {self.intermittent_distance_x} - {self.intermittent_distance_y}')

                elif ( not self.y_achieved ) and self.y_rotate_achieved and self.forward_obstract_distance[1] == 0.0 and self.forward_obstract_distance[2] != 0.0 and ( round(self.vehicle_local_position.y,2) > round(self.forward_obstract_distance[2],2)-0.25 and round(self.vehicle_local_position.y,2) < round(self.forward_obstract_distance[2],2)+0.25 ) :
                    self.get_logger().info(f'y Achieved at mid point using y, obstacle ahead - {self.vehicle_local_position.x} - {self.vehicle_local_position.y}')
                    if not self.check_destination( "y" ) :
                        self.forward_obstract_distance[0] = "y"
                        self.obstacle_found = True
                elif ( not self.y_achieved ) and self.y_rotate_achieved and self.forward_obstract_distance[1] == 0.0 and self.forward_obstract_distance[2] == 0.0 and ( round(self.vehicle_local_position.y,2) <= round(self.forward_distance_y,2)-0.25 or round(self.vehicle_local_position.y,2) >= round(self.forward_distance_y,2)+0.25 ):

                    y_diff = self.forward_distance_y - self.vehicle_local_position.y

                    if ( y_diff < -1 ) or ( y_diff > 1 ) :
                        y_diff = np.sign(y_diff)

                    self.intermittent_distance_y = self.vehicle_local_position.y+y_diff

                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    self.get_logger().info(f'y Going towards required y - {self.intermittent_distance_x} - {self.intermittent_distance_y}')

                elif ( not self.y_achieved ) and self.y_rotate_achieved and self.forward_obstract_distance[1] == 0.0 and self.forward_obstract_distance[2] == 0.0 and ( round(self.vehicle_local_position.y,2) > round(self.forward_distance_y,2)-0.25 and round(self.vehicle_local_position.y,2) < round(self.forward_distance_y,2)+0.25 ) :
                    self.get_logger().info(f"y {self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.y_achieved = True

                    self.get_logger().info(f"Before : {( self.forward_distance_x - self.vehicle_local_position.x )} - {self.yaw_angle}")
                    # if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0  and abs(self.yaw_angle) == 1.57079:
                    #     self.yaw_angle = 0.0
                    # elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0  and abs(self.yaw_angle) == 1.57079 :
                    #     self.yaw_angle = 3.14158

                    if self.actual_angle == -1.57079 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle += 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle-= 1.57079
                    elif self.actual_angle == 1.57079 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle -= 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle += 1.57079
                    elif self.actual_angle == 4.71237 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle += 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle -= 1.57079
                    elif self.actual_angle == -4.71237 :
                        if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                            self.actual_angle -= 1.57079
                        elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                            self.actual_angle += 1.57079

                    if self.actual_angle > 0 and self.actual_angle > 3.14158 :
                        self.yaw_angle = self.actual_angle - 6.28316
                        self.actual_angle_difference = 6.28316
                    elif self.actual_angle < 0 and self.actual_angle < -3.14158 :
                        self.yaw_angle = 6.28316 + self.actual_angle
                        self.actual_angle_difference = -6.28316
                    else :
                        self.yaw_angle = self.actual_angle
                        self.actual_angle_difference = 0.0

                    if abs(self.yaw_angle) == 0.0 :
                        self.actual_angle_difference = 0.0
                        self.actual_angle = 0.0

                    self.get_logger().info(f"After : {( self.forward_distance_x - self.vehicle_local_position.x )} - {self.yaw_angle} - {self.actual_angle_difference} - {self.actual_angle}")
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                    self.intermittent_distance_x = self.vehicle_local_position.x
                    self.intermittent_distance_y = self.vehicle_local_position.y
                    if ( self.vehicle_local_position.x < self.forward_distance_x-0.25 ) or ( self.vehicle_local_position.x > self.forward_distance_x+0.25 ) :
                        self.x_achieved = False
                        self.x_rotate_achieved = False
                        self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    elif not self.traverse_coordinates_queue.empty() :
                        self.x_achieved = False
                        self.x_rotate_achieved = False
                        self.forward_distance_x, self.forward_distance_y, self.takeoff_height = self.traverse_coordinates_queue.get()
                        self.get_logger().info(f"New Position has been set to traverse - {self.forward_distance_x} - {self.forward_distance_y} - {self.takeoff_height}")
                    else :
                        self.land()
                        convert( self.log_file_name )
                        exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def check_destination( self, direction ) :
        if direction == "x" :
            if ( not self.x_achieved ) and self.x_rotate_achieved and ( round(self.vehicle_local_position.x,2) > round(self.forward_distance_x,2)-0.25 and round(self.vehicle_local_position.x,2) < round(self.forward_distance_x,2)+0.25 ) :
                self.get_logger().info(f"x {self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                self.x_achieved = True

                self.get_logger().info(f"Before : {( self.forward_distance_y - self.vehicle_local_position.y )} - {self.yaw_angle}")
                # if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0  and abs(self.yaw_angle) == 0.0:
                #     self.yaw_angle = 1.57079
                # elif ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0  and abs(self.yaw_angle) == 3.14158 :
                #     self.yaw_angle = 1.57079
                # elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0  and abs(self.yaw_angle) == 0.0 :
                #     self.yaw_angle = -1.57079
                # elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0  and abs(self.yaw_angle) == 3.14158 :
                #     self.yaw_angle = -1.57079

                
                if self.actual_angle == 0.0 :
                    if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0 :
                        self.actual_angle += 1.57079
                    elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0 :
                        self.actual_angle-= 1.57079
                elif self.actual_angle == 3.14158 :
                    if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0 :
                        self.actual_angle -= 1.57079
                    elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0 :
                        self.actual_angle += 1.57079
                elif self.actual_angle == -3.14158 :
                    if ( self.forward_distance_y - self.vehicle_local_position.y ) >= 0 :
                        self.actual_angle -= 1.57079
                    elif ( self.forward_distance_y - self.vehicle_local_position.y ) < 0 :
                        self.actual_angle += 1.57079

                if self.actual_angle > 0 and self.actual_angle > 3.14158 :
                    self.yaw_angle = self.actual_angle - 6.28316
                    self.actual_angle_difference = 6.28316
                elif self.actual_angle < 0 and self.actual_angle < -3.14158 :
                    self.yaw_angle = 6.28316 + self.actual_angle
                    self.actual_angle_difference = -6.28316
                else :
                    self.yaw_angle = self.actual_angle
                    self.actual_angle_difference = 0.0

                if abs(self.yaw_angle) == 0.0 :
                    self.actual_angle_difference = 0.0
                    self.actual_angle = 0.0

                self.get_logger().info(f"After : {( self.forward_distance_y - self.vehicle_local_position.y )} - {self.yaw_angle} - {self.actual_angle_difference} - {self.actual_angle}")
                self.forward_obstract_distance[1] = 0.0
                self.forward_obstract_distance[2] = 0.0
                self.intermittent_distance_x = self.vehicle_local_position.x
                self.intermittent_distance_y = self.vehicle_local_position.y
                if ( self.vehicle_local_position.y < self.forward_distance_y-0.25 ) or ( self.vehicle_local_position.y > self.forward_distance_y+0.25 ) :
                    self.y_achieved = False
                    self.y_rotate_achieved = False
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    return True
                elif not self.traverse_coordinates_queue.empty() :
                    self.y_achieved = False
                    self.y_rotate_achieved = False
                    self.forward_distance_x, self.forward_distance_y, self.takeoff_height = self.traverse_coordinates_queue.get()
                    self.get_logger().info(f"New Position has been set to traverse - {self.forward_distance_x} - {self.forward_distance_y} - {self.takeoff_height}")
                else :
                    self.land()
                    convert( self.log_file_name )
                    exit(0)
        elif direction == "y" :
            if ( not self.y_achieved ) and self.y_rotate_achieved and ( round(self.vehicle_local_position.y,2) > round(self.forward_distance_y,2)-0.25 and round(self.vehicle_local_position.y,2) < round(self.forward_distance_y,2)+0.25 ) :
                self.get_logger().info(f"y {self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                self.y_achieved = True
                self.get_logger().info(f"Before : {( self.forward_distance_x - self.vehicle_local_position.x )} - {self.yaw_angle}")
                # if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0  and abs(self.yaw_angle) == 1.57079:
                #     self.yaw_angle = 0.0
                # elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0  and abs(self.yaw_angle) == 1.57079 :
                #     self.yaw_angle = 3.14158

                if self.actual_angle == -1.57079 :
                    if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                        self.actual_angle += 1.57079
                    elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                        self.actual_angle-= 1.57079
                elif self.actual_angle == 1.57079 :
                    if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                        self.actual_angle -= 1.57079
                    elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                        self.actual_angle += 1.57079
                elif self.actual_angle == 4.71237 :
                    if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                        self.actual_angle += 1.57079
                    elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                        self.actual_angle -= 1.57079
                elif self.actual_angle == -4.71237 :
                    if ( self.forward_distance_x - self.vehicle_local_position.x ) >= 0 :
                        self.actual_angle -= 1.57079
                    elif ( self.forward_distance_x - self.vehicle_local_position.x ) < 0 :
                        self.actual_angle += 1.57079

                if self.actual_angle > 0 and self.actual_angle > 3.14158 :
                    self.yaw_angle = self.actual_angle - 6.28316
                    self.actual_angle_difference = 6.28316
                elif self.actual_angle < 0 and self.actual_angle < -3.14158 :
                    self.yaw_angle = 6.28316 + self.actual_angle
                    self.actual_angle_difference = -6.28316
                else :
                    self.yaw_angle = self.actual_angle
                    self.actual_angle_difference = 0.0

                if abs(self.yaw_angle) == 0.0 :
                    self.actual_angle_difference = 0.0
                    self.actual_angle = 0.0

                self.get_logger().info(f"After : {( self.forward_distance_x - self.vehicle_local_position.x )} - {self.yaw_angle} - {self.actual_angle_difference} - {self.actual_angle}")
                self.forward_obstract_distance[1] = 0.0
                self.forward_obstract_distance[2] = 0.0
                self.intermittent_distance_x = self.vehicle_local_position.x
                self.intermittent_distance_y = self.vehicle_local_position.y
                if ( self.vehicle_local_position.x < self.forward_distance_x-0.25 ) or ( self.vehicle_local_position.x > self.forward_distance_x+0.25 ) :
                    self.x_achieved = False
                    self.x_rotate_achieved = False
                    self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height,self.yaw_angle)
                    return True
                elif not self.traverse_coordinates_queue.empty() :
                    self.x_achieved = False
                    self.x_rotate_achieved = False
                    self.forward_distance_x, self.forward_distance_y, self.takeoff_height = self.traverse_coordinates_queue.get()
                    self.get_logger().info(f"New Position has been set to traverse - {self.forward_distance_x} - {self.forward_distance_y} - {self.takeoff_height}")
                else :
                    self.land()
                    convert( self.log_file_name )
                    exit(0)
        return False


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    # plt.ion()
    # plt.figure(figsize=(200,200))
    # plt.show()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)