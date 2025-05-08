#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, ObstacleDistance,VehicleAttitude
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import shutil
# import matplotlib
# matplotlib.use('QtAgg')
import json


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

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
            ObstacleDistance, '/fmu/out/obstacle_distance', self.obstacle_distance_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -10.0
        self.forward_distance_x = 10.0
        self.forward_obstract_distance = [ "x" , 0.0, 0.0 ]
        self.continue_direction = [0.0,0.0]
        self.forward_distance_y = 10.0
        self.vehicle_step_distance = 0.0
        self.safe_distance_from_qudacopter = 60
        self.square_check = 0
        self.obstacle_distance = ObstacleDistance()
        self.obstacle_found = False
        self.obstract_distance_x = 0.0
        self.obstract_distance_y = 0.0
        self.obstract_distance_z = 0.0
        # self.current_heading = 0.0
        self.z_achieved = False
        self.x_achieved = False
        self.y_achieved = False
        self.yaw_angle = 1.57079
        self.previous_front_obstacle_found = 0.0
        self.previous_heading = 0.0
        self.x_rotate_achieved = False
        self.y_rotate_achieved = False

        self.x_data = []
        self.y_data = []
        # self.z_data = []
        self.drone_x_data = []
        self.drone_y_data = []
        # self.drone_z_data = []
        # self.fig, self.ax = plt.subplots()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.previous_heading = self.vehicle_local_position.heading
        self.vehicle_local_position = vehicle_local_position
        self.drone_x_data.append(-1*self.vehicle_local_position.x)
        self.drone_y_data.append(self.vehicle_local_position.y)
        # self.drone_z_data.append(-1*self.vehicle_local_position.z)
        # self.get_logger().info(f'vehicle_local_position : {self.vehicle_local_position}') 

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def lidar_processing( self , lidar_msg) :

        if self.z_achieved :
            obstacle_distances = [(distance_idx,lidar_msg.distances[distance_idx]) for distance_idx in range(len(lidar_msg.distances)) if lidar_msg.distances[distance_idx] < 599]

            # self.get_logger().info(f'obstacle_distances - {obstacle_distances}')
            for obstacle_distance in obstacle_distances :
                if math.radians(45) + math.radians(90) - self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*lidar_msg.increment) < 0 :
                    radians = math.radians(360) + math.radians(45) + math.radians(90) - self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*lidar_msg.increment)
                else :
                    radians = math.radians(45) + math.radians(90) - self.vehicle_local_position.heading - math.radians(obstacle_distance[0]*lidar_msg.increment)
                # if 45 + 90 - self.yaw_angle_degree + (-1*obstacle_distance[0]*lidar_msg.increment) < 0 :
                #     radians = math.radians(360 + 45 + 90 - self.yaw_angle_degree + (-1*(obstacle_distance[0]*lidar_msg.increment)))
                # else :
                #     radians = math.radians(45 + 90 - self.yaw_angle_degree + (-1*(obstacle_distance[0]*lidar_msg.increment)))
                cos_value = np.cos(radians)
                sin_value = np.sin(radians)
                relative_obstacle_x = (obstacle_distance[1]/100)*cos_value
                relative_obstacle_y = (obstacle_distance[1]/100)*sin_value
                absolute_obstacle_x = relative_obstacle_x + (-1*self.vehicle_local_position.x)
                absolute_obstacle_y = relative_obstacle_y + self.vehicle_local_position.y
                # absolute_obstacle_z = -1*self.vehicle_local_position.z

                self.x_data.append(absolute_obstacle_x)
                self.y_data.append(absolute_obstacle_y)
                # self.z_data.append(absolute_obstacle_z)

            # self.get_logger().info(f'lidar distance - {lidar_msg.distances}')
            # self.get_logger().info(f'lidar angle - {[idx*lidar_msg.increment for idx in range(len(lidar_msg.distances))]}')
            # self.get_logger().info(f'lidar angle - {[360 + 45 + 90 - self.yaw_angle_degree + (-1*(idx*lidar_msg.increment)) if 45 + 90 - self.yaw_angle_degree + (-1*idx*lidar_msg.increment) < 0 else 45 + 90 - self.yaw_angle_degree + (-1*idx*lidar_msg.increment) for idx in range(len(lidar_msg.distances))]}')
            # self.get_logger().info(f'lidar angle - {[math.degrees(6.28319 + 0.785398 + 1.5708 - self.vehicle_local_position.heading - - math.radians(idx*lidar_msg.increment)) if 0.785398 + 1.5708 - self.vehicle_local_position.heading - math.radians(idx*lidar_msg.increment) < 0 else math.degrees(0.785398 + 1.5708 - self.vehicle_local_position.heading - math.radians(idx*lidar_msg.increment)) for idx in range(len(lidar_msg.distances))]}')
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
            # self.axes.scatter(vechile_position_x,vechile_position_y,c="#fd72c0",marker=".",s=5)

    # def update_plot(self):
    #     self.ax.clear()
    #     self.ax.scatter(self.x_data, self.y_data,c="#fd72c0",marker=".",s=5)
    #     # self.ax.set_xlabel("Time/Index")
    #     # self.ax.set_ylabel("Data Value")
    #     self.fig.canvas.draw()
    #     plt.pause(0.001)

    
    def obstacle_distance_callback(self, msg):
        """
        This function is called when new data is received on the topic.
        """
        self.get_logger().info(f'Obstacle distance: {msg.distances} - {self.previous_heading} - {self.vehicle_local_position.heading}')  # Print the distance
        # Add your code here to process the obstacle distance data
        all_angles_distance = msg.distances
        self.lidar_processing( msg )
        # self.x_data.append(self.vehicle_local_position.x)
        # self.y_data.append(self.vehicle_local_position.y)
        # self.update_plot()
        # self.get_logger().info(f'All Obstacle distance: {msg}') 
        back_obstacle_found,left_obstacle_found,front_obstacle_found,right_obstacle_found,left_minimum,right_minimum,back_minimum,front_minimum,right_up_corner_distances_minimum,right_down_corner_distances_minimum,left_down_corner_distances_minimum,left_up_corner_distances_minimum = self.obstacle_and_direction( msg , 300)
        back_obstacle_found_5,left_obstacle_found_5,front_obstacle_found_5,right_obstacle_found_5,_,_,_,_,_,_,_,_ = self.obstacle_and_direction( msg , 500)
        # if ( not self.obstacle_found ) and front_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
        #     if not self.x_achieved :

        #         if self.forward_distance_x < 0 and self.vehicle_local_position.x > self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x-0.25
        #         elif self.forward_distance_x < 0 and self.vehicle_local_position.x < self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x+0.25
        #         elif self.forward_distance_x > 0 and self.vehicle_local_position.x > self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x-0.25
        #         elif self.forward_distance_x > 0 and self.vehicle_local_position.x < self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x+0.25

        #     elif not self.y_achieved :
        #         # self.forward_obstract_distance[1] = self.vehicle_local_position.y-1

        #         if self.forward_distance_y < 0 and self.vehicle_local_position.y > self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y-0.25
        #         elif self.forward_distance_y < 0 and self.vehicle_local_position.y < self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y+0.25
        #         elif self.forward_distance_y > 0 and self.vehicle_local_position.y > self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y-0.25
        #         elif self.forward_distance_y > 0 and self.vehicle_local_position.y < self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y+0.25
            
        #     self.get_logger().info(f'forward Corner Obstacle between traversing : {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {self.forward_obstract_distance[1]} - {self.x_achieved} - {self.y_achieved}')
        # elif ( not self.obstacle_found ) and back_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
        #     if not self.x_achieved :

        #         if self.forward_distance_x < 0 and self.vehicle_local_position.x > self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x-1
        #         elif self.forward_distance_x < 0 and self.vehicle_local_position.x < self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x+1
        #         elif self.forward_distance_x > 0 and self.vehicle_local_position.x > self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x-1
        #         elif self.forward_distance_x > 0 and self.vehicle_local_position.x < self.forward_distance_x:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.x+1

        #     elif not self.y_achieved :
        #         # self.forward_obstract_distance[1] = self.vehicle_local_position.y-1

        #         if self.forward_distance_y < 0 and self.vehicle_local_position.y > self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y-1
        #         elif self.forward_distance_y < 0 and self.vehicle_local_position.y < self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y+1
        #         elif self.forward_distance_y > 0 and self.vehicle_local_position.y > self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y-1
        #         elif self.forward_distance_y > 0 and self.vehicle_local_position.y < self.forward_distance_y:
        #             self.forward_obstract_distance[1] = self.vehicle_local_position.y+1
            
        #     self.get_logger().info(f'Back Corner Obstacle between traversing : {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {self.forward_obstract_distance[1]} - {self.x_achieved} - {self.y_achieved}')
        if ( not self.obstacle_found ) and self.forward_obstract_distance[1] == 0.0 and ( front_obstacle_found_5 >= 2 ) and (( self.forward_distance_x > 4 ) or ( self.forward_distance_y > 4 ) ) :
            if not self.x_achieved :

                if self.forward_distance_x < 0 and self.vehicle_local_position.x > self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x-1
                elif self.forward_distance_x < 0 and self.vehicle_local_position.x < self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x+1
                elif self.forward_distance_x > 0 and self.vehicle_local_position.x > self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x-1
                elif self.forward_distance_x > 0 and self.vehicle_local_position.x < self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x+1

            elif not self.y_achieved :
                # self.forward_obstract_distance[1] = self.vehicle_local_position.y-1

                if self.forward_distance_y < 0 and self.vehicle_local_position.y > self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y-1
                elif self.forward_distance_y < 0 and self.vehicle_local_position.y < self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y+1
                elif self.forward_distance_y > 0 and self.vehicle_local_position.y > self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y-1
                elif self.forward_distance_y > 0 and self.vehicle_local_position.y < self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y+1

            self.get_logger().info(f'Obstacle between traversing : {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {self.forward_obstract_distance[1]} - {self.x_achieved} - {self.y_achieved}')
        elif ( not self.obstacle_found ) and self.forward_obstract_distance[2] == 0.0 and ( front_obstacle_found_5 >= 2 ) and front_minimum < ( self.safe_distance_from_qudacopter*2 ) :
            if not self.x_achieved :

                if self.forward_distance_x < 0 and self.vehicle_local_position.x > self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x-0.05
                elif self.forward_distance_x < 0 and self.vehicle_local_position.x < self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x+0.05
                elif self.forward_distance_x > 0 and self.vehicle_local_position.x > self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x-0.05
                elif self.forward_distance_x > 0 and self.vehicle_local_position.x < self.forward_distance_x:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.x+0.05

            elif not self.y_achieved :
                # self.forward_obstract_distance[1] = self.vehicle_local_position.y-1

                if self.forward_distance_y < 0 and self.vehicle_local_position.y > self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y-0.05
                elif self.forward_distance_y < 0 and self.vehicle_local_position.y < self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y+0.05
                elif self.forward_distance_y > 0 and self.vehicle_local_position.y > self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y-0.05
                elif self.forward_distance_y > 0 and self.vehicle_local_position.y < self.forward_distance_y:
                    self.forward_obstract_distance[1] = self.vehicle_local_position.y+0.05
            
            self.forward_obstract_distance[2] = 1

            self.get_logger().info(f'Obstacle between traversing : {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {self.forward_obstract_distance[1]} - {self.x_achieved} - {self.y_achieved}')
        elif self.obstacle_found :
            if self.forward_obstract_distance[0] == "x" :
                self.get_logger().info(f'4m obstacles : {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {all_angles_distance}')
                # if left_obstacle_found < 2 and (( right_obstacle_found >= 2 ) or ( right_obstacle_found < 2 and left_obstacle_found <= right_obstacle_found ) ) and front_obstacle_found >= 2:
                #     self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+1, self.vehicle_local_position.z,self.yaw_angle)
                # elif right_obstacle_found < 2 and (( left_obstacle_found >= 2 ) or ( left_obstacle_found < 2 and right_obstacle_found <= left_obstacle_found ) ) and front_obstacle_found >= 2:
                #     self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y-1, self.vehicle_local_position.z,self.yaw_angle)
                # elif front_obstacle_found < 1 :
                #     self.get_logger().info(f'Forward obstacle cleared')
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
                    if front_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x-0.25,self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'Right down corner - Moving up in -0.25 direction - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                    elif back_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x-0.25,self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'Up corner - Moving down in +0.25 direction - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                    else :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+self.continue_direction[1], self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'Continue in {self.continue_direction[1]} direction - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found <= right_obstacle_found :
                    self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+self.continue_direction[1], self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = -0.25
                    else :
                        self.continue_direction[1] = -1.0
                    self.get_logger().info(f'Going left - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found >= right_obstacle_found :
                    self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+self.continue_direction[1], self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = 0.25
                    else :
                        self.continue_direction[1] = 1.0
                    self.get_logger().info(f'Going right - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found >= 2 and left_obstacle_found == right_obstacle_found and self.continue_direction[1] != 0.0 :
                    self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+self.continue_direction[1], self.vehicle_local_position.z,self.yaw_angle)
                    self.get_logger().info(f'Continue in {self.continue_direction[1]} direction - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                elif left_obstacle_found < 2 and (( right_obstacle_found >= 2 ) or ( right_obstacle_found < 2 and left_obstacle_found <= right_obstacle_found ) ) and front_obstacle_found >= 2:
                    self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+self.continue_direction[1], self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = -0.25
                    else :
                        self.continue_direction[1] = -1.0
                    self.get_logger().info(f'Going left - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                elif right_obstacle_found < 2 and (( left_obstacle_found >= 2 ) or ( left_obstacle_found < 2 and right_obstacle_found <= left_obstacle_found ) ) and front_obstacle_found >= 2:
                    self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+self.continue_direction[1], self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[1] = 0.25
                    else :
                        self.continue_direction[1] = 1.0
                    self.get_logger().info(f'Going right - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                elif front_obstacle_found < 1 and left_minimum >= self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter and back_minimum >= self.safe_distance_from_qudacopter:
                    self.get_logger().info(f'Forward obstacle cleared - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                    self.obstacle_found = False
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                else :
                    if self.continue_direction[1] != 0.0 :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+self.continue_direction[1], self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'Continue in {self.continue_direction[1]} direction - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                    else :
                        self.get_logger().info(f'Continue direction not set')

            
            elif self.forward_obstract_distance[0] == "y" :
                self.get_logger().info(f'y 4m obstacles : {back_obstacle_found} - {left_obstacle_found} - {front_obstacle_found} - {right_obstacle_found} - {all_angles_distance}')
                if front_obstacle_found > 0 and front_obstacle_found <= self.previous_front_obstacle_found and self.continue_direction[0] != 0.0 and left_minimum >= self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter and back_minimum >= self.safe_distance_from_qudacopter:
                    if ( left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 )) and abs(self.continue_direction[0]) == 1 :
                        self.continue_direction[0] = self.continue_direction[0]/4
                    elif left_minimum >= self.safe_distance_from_qudacopter*2 and right_minimum >= self.safe_distance_from_qudacopter*2 and back_minimum >= self.safe_distance_from_qudacopter*2 and abs(self.continue_direction[0]) == 0.25 :
                        self.continue_direction[0] = self.continue_direction[0]*4
                    if front_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_down_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y+0.25, self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'y Right down corner - Moving up in +0.25 direction - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                    elif back_minimum > ( self.safe_distance_from_qudacopter*2 ) and ( (right_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 )) or (left_up_corner_distances_minimum < ( self.safe_distance_from_qudacopter*2 ))) :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y-0.25, self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'y Up corner - Moving down in -0.25 direction - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}')
                    else :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x+self.continue_direction[0],self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'y Continue in {self.continue_direction[0]} direction - {all_angles_distance}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found <= right_obstacle_found :
                    self.publish_position_setpoint("position", self.vehicle_local_position.x-1,self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = -0.25
                    else :
                        self.continue_direction[0] = -1.0
                    self.get_logger().info(f'Going left - {all_angles_distance}')
                elif front_obstacle_found >= 2 and left_obstacle_found >= 2 and right_obstacle_found >= 2 and left_obstacle_found >= right_obstacle_found :
                    self.publish_position_setpoint("position", self.vehicle_local_position.x+1,self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = 0.25
                    else :
                        self.continue_direction[0] = 1.0
                    self.get_logger().info(f'Going right - {all_angles_distance}')
                elif front_obstacle_found >= 2 and left_obstacle_found == right_obstacle_found and self.continue_direction[0] != 0.0 :
                    self.publish_position_setpoint("position", self.vehicle_local_position.x+self.continue_direction[0],self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                    self.get_logger().info(f'Continue in {self.continue_direction[0]} direction - {all_angles_distance}')
                elif left_obstacle_found < 2 and (( right_obstacle_found >= 2 ) or ( right_obstacle_found < 2 and left_obstacle_found <= right_obstacle_found ) ) and front_obstacle_found >= 2:
                    self.publish_position_setpoint("position", self.vehicle_local_position.x-1,self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = -0.25
                    else :
                        self.continue_direction[0] = -1.0
                    self.get_logger().info(f'Going left - {all_angles_distance}')
                elif right_obstacle_found < 2 and (( left_obstacle_found >= 2 ) or ( left_obstacle_found < 2 and right_obstacle_found <= left_obstacle_found ) ) and front_obstacle_found >= 2:
                    self.publish_position_setpoint("position", self.vehicle_local_position.x+1,self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                    if left_minimum < ( self.safe_distance_from_qudacopter*2 ) or right_minimum < ( self.safe_distance_from_qudacopter*2 ) or back_minimum < ( self.safe_distance_from_qudacopter*2 ) :
                        self.continue_direction[0] = 0.25
                    else :
                        self.continue_direction[0] = 1.0
                    self.get_logger().info(f'Going right - {all_angles_distance}')
                elif front_obstacle_found < 1 and left_minimum >= self.safe_distance_from_qudacopter and right_minimum >= self.safe_distance_from_qudacopter and back_minimum >= self.safe_distance_from_qudacopter:
                    self.get_logger().info(f'Forward obstacle cleared')
                    self.obstacle_found = False
                    self.forward_obstract_distance[1] = 0.0
                    self.forward_obstract_distance[2] = 0.0
                else :
                    if self.continue_direction[0] != 0.0 :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x+self.continue_direction[0],self.vehicle_local_position.y, self.vehicle_local_position.z,self.yaw_angle)
                        self.get_logger().info(f'Continue in {self.continue_direction[0]} direction - {all_angles_distance}')
                    else :
                        self.get_logger().info(f'Continue direction not set')
        self.previous_front_obstacle_found = front_obstacle_found

    def obstacle_and_direction( self, msg, threshold ):
        msg = msg.distances.tolist()
        obstacle_distances_right = msg[5:13] #msg[:18] #msg[:8]+msg[-8:]
        obstacle_distances_back = msg[23:31] #msg[8:34] 
        obstacle_distances_left = msg[41:49] #msg[34:50]
        obstacle_distances_front = msg[59:67]

        right_up_corner_distances = msg[-2:]+msg[:2]
        right_down_corner_distances = msg[16:20]
        left_down_corner_distances = msg[34:38]
        left_up_corner_distances = msg[52:56]
        # obstacle_distances_centre = msg[61:65]
        back_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_back)])
        left_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_left)])
        front_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_front)])
        right_obstacle_found = len([*filter(lambda x: x < threshold, obstacle_distances_right)])
        left_minimum = min(obstacle_distances_left)
        right_minimum = min(obstacle_distances_right)
        back_minimum = min(obstacle_distances_back)
        front_minimum = min(obstacle_distances_front)
        right_up_corner_distance_minimum = min(right_up_corner_distances)
        right_down_corner_distances_minimum = min(right_down_corner_distances)
        left_down_corner_distances_minimum = min(left_down_corner_distances)
        left_up_corner_distances_minimum = min(left_up_corner_distances)
        return back_obstacle_found,left_obstacle_found,front_obstacle_found,right_obstacle_found,left_minimum,right_minimum,back_minimum,front_minimum,right_up_corner_distance_minimum,right_down_corner_distances_minimum,left_down_corner_distances_minimum,left_up_corner_distances_minimum

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

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        if not self.obstacle_found :
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
                    self.yaw_angle -= 1.57079
                    # time.sleep(30)

            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 1:

                if ( not self.x_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.x,0) != round(self.forward_obstract_distance[1],0) :
                    if self.forward_obstract_distance[1] < 0 and self.vehicle_local_position.x > self.forward_obstract_distance[1]:
                        self.publish_position_setpoint("position", self.vehicle_local_position.x-1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                    elif self.forward_obstract_distance[1] < 0 and self.vehicle_local_position.x < self.forward_obstract_distance[1] :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x+1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                    elif self.forward_obstract_distance[1] > 0 and self.vehicle_local_position.x > self.forward_obstract_distance[1] :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x-1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                    elif self.forward_obstract_distance[1] > 0 and self.vehicle_local_position.x < self.forward_obstract_distance[1] :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x+1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif ( not self.x_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.x,0) == round(self.forward_obstract_distance[1],0) :
                    self.get_logger().info(f'Achieved at mid point , obstacle ahead')
                    self.forward_obstract_distance[0] = "x"
                    self.obstacle_found = True
                elif ( not self.x_achieved ) and round(self.vehicle_local_position.x,0) != round(self.forward_distance_x,0) :

                    if self.forward_distance_x < 0 and self.vehicle_local_position.x > self.forward_distance_x:
                        self.publish_position_setpoint("position", self.vehicle_local_position.x-1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                    elif self.forward_distance_x < 0 and self.vehicle_local_position.x < self.forward_distance_x :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x+1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                    elif self.forward_distance_x > 0 and self.vehicle_local_position.x > self.forward_distance_x :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x-1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                    elif self.forward_distance_x > 0 and self.vehicle_local_position.x < self.forward_distance_x :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x+1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)

                    # self.publish_position_setpoint("position", self.vehicle_local_position.x+1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif ( not self.x_achieved ) and round(self.vehicle_local_position.x,0) == round(self.forward_distance_x,0) :
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.x_achieved = True
                    # self.publish_offboard_control_heartbeat_signal("rotate")
                    # self.yaw_angle += 1.57079
                #     self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif self.x_achieved and ( not self.x_rotate_achieved ) and round(self.vehicle_local_position.heading,2) < round(self.yaw_angle,2)-0.05:
                    self.get_logger().info(f"rotating - {self.vehicle_local_position.heading} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.publish_offboard_control_heartbeat_signal("rotate")
                #     self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif self.x_achieved and ( not self.x_rotate_achieved ) and round(self.vehicle_local_position.heading,2) > round(self.yaw_angle,2)-0.05 and round(self.vehicle_local_position.heading,2) < round(self.yaw_angle,2)+0.05 :
                    self.get_logger().info(f"{self.square_check} {self.vehicle_local_position.heading} rotate completed - {self.vehicle_local_position.heading} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.x_rotate_achieved = True
                    self.yaw_angle += 1.57079
                    self.forward_obstract_distance[1] = 0.0
                    # self.square_check += 1

                # self.publish_position_setpoint("position", self.forward_distance_x, 0.0, self.takeoff_height)
                # if round(self.vehicle_local_position.x,0) == self.forward_distance_x :
                #     self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.square_check += 1
            # elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 2:


                elif ( not self.y_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.y,0) != round(self.forward_obstract_distance[1],0) :
                    # self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)

                    if self.forward_obstract_distance[1] < 0 and self.vehicle_local_position.y > self.forward_obstract_distance[1]:
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y-1, self.takeoff_height,self.yaw_angle)
                    elif self.forward_obstract_distance[1] < 0 and self.vehicle_local_position.y < self.forward_obstract_distance[1] :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)
                    elif self.forward_obstract_distance[1] > 0 and self.vehicle_local_position.y > self.forward_obstract_distance[1] :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y-1, self.takeoff_height,self.yaw_angle)
                    elif self.forward_obstract_distance[1] > 0 and self.vehicle_local_position.y < self.forward_obstract_distance[1] :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)

                elif ( not self.y_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.y,0) == round(self.forward_obstract_distance[1],0) :
                    self.get_logger().info(f'Achieved at mid point , obstacle ahead')
                    self.forward_obstract_distance[0] = "y"
                    self.obstacle_found = True
                elif ( not self.y_achieved ) and round(self.vehicle_local_position.y,0) != round(self.forward_distance_y,0) :

                    if self.forward_distance_y < 0 and self.vehicle_local_position.y > self.forward_distance_y:
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y-1, self.takeoff_height,self.yaw_angle)
                    elif self.forward_distance_y < 0 and self.vehicle_local_position.y < self.forward_distance_y :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)
                    elif self.forward_distance_y > 0 and self.vehicle_local_position.y > self.forward_distance_y :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y-1, self.takeoff_height,self.yaw_angle)
                    elif self.forward_distance_y > 0 and self.vehicle_local_position.y < self.forward_distance_y :
                        self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)

                    # self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y+1, self.takeoff_height,self.yaw_angle)
                elif ( not self.y_achieved ) and round(self.vehicle_local_position.y,0) == round(self.forward_distance_y,0) :
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.y_achieved = True
                    # if ( self.vehicle_local_position.x < self.forward_distance_x-0.2 ) or ( self.vehicle_local_position.x > self.forward_distance_x+0.2 ) :
                    #     self.x_achieved = False
                    #     self.x_rotate_achieved = False
                #     self.publish_offboard_control_heartbeat_signal("rotate")
                    # self.yaw_angle -= 1.57079
                #     self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif self.y_achieved and ( not self.y_rotate_achieved ) and round(self.vehicle_local_position.heading,2) < (round(self.yaw_angle,2)-0.05):
                    self.get_logger().info(f"y rotating - {round(self.vehicle_local_position.heading,2)} - {round(self.yaw_angle,2)} - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.publish_offboard_control_heartbeat_signal("rotate")
                #     self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
                elif self.y_achieved and ( not self.y_rotate_achieved ) and round(self.vehicle_local_position.heading,2) > (round(self.yaw_angle,2)-0.05) and round(self.vehicle_local_position.heading,2) < (round(self.yaw_angle,2)+0.05) :
                    self.get_logger().info(f"{self.square_check} {self.vehicle_local_position.heading} rotate completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    # self.square_check += 1
                    self.y_rotate_achieved = True
                    self.yaw_angle -= 1.57079
                    self.forward_obstract_distance[1] = 0.0
                    if ( self.vehicle_local_position.x < self.forward_distance_x-0.2 ) or ( self.vehicle_local_position.x > self.forward_distance_x+0.2 ) :
                        self.x_achieved = False
                        self.x_rotate_achieved = False

                # self.publish_position_setpoint("position", self.forward_distance_x, self.forward_distance_y, self.takeoff_height,self.yaw_angle)
                # if round(self.vehicle_local_position.y,0) == self.forward_distance_y :
                #     self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.square_check += 1
            # elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 3:

            #     if ( not self.x_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.x,0) != round(self.forward_obstract_distance[1],0) :
            #         self.publish_position_setpoint("position", self.vehicle_local_position.x-1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
            #     elif ( not self.x_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.x,0) == round(self.forward_obstract_distance[1],0) :
            #         self.get_logger().info(f'Achieved at mid point , obstacle ahead')
            #         self.forward_obstract_distance[0] = "x"
            #         self.obstacle_found = True
            #     elif ( not self.x_achieved ) and round(self.vehicle_local_position.x,0) != 0.0 :
            #         self.publish_position_setpoint("position", self.vehicle_local_position.x-1, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
            #     elif ( not self.x_achieved ) and round(self.vehicle_local_position.x,0) == 0.0 :
            #         self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
            #         self.x_achieved = True
            #         self.y_achieved = False
            #         self.publish_offboard_control_heartbeat_signal("rotate")
            #         self.yaw_angle += 1.57079
            #         self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
            #     elif self.x_achieved and round(self.vehicle_local_position.heading,2) != round(-1.57079,2):
            #         # self.get_logger().info(f"rotating - {self.vehicle_local_position.heading} - {self.vehicle_local_position.heading_var} - {self.vehicle_local_position.unaided_heading}")
            #         self.publish_offboard_control_heartbeat_signal("rotate")
            #         self.publish_position_setpoint("rotate", self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height,self.yaw_angle)
            #     elif self.x_achieved and round(self.vehicle_local_position.heading,2) == round(-1.57079,2) :
            #         self.get_logger().info(f"{self.square_check} {self.vehicle_local_position.heading} rotate completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
            #         self.square_check += 1

            #     # self.publish_position_setpoint("position", 0.0, self.forward_distance_y, self.takeoff_height,self.yaw_angle)
            #     # if round(self.vehicle_local_position.x,0) == 0.0 :
            #     #     self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
            #     #     self.square_check += 1
            # elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 4:

            #     self.get_logger().info(f"{self.y_achieved} - {self.forward_obstract_distance[1]} - {self.vehicle_local_position.y}")
            #     if ( not self.y_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.y,0) != round(self.forward_obstract_distance[1],0) :
            #         self.publish_position_setpoint("position", self.vehicle_local_position.x, self.vehicle_local_position.y-1, self.takeoff_height,self.yaw_angle)
            #     elif ( not self.y_achieved ) and self.forward_obstract_distance[1] != 0.0 and round(self.vehicle_local_position.y,0) == round(self.forward_obstract_distance[1],0) :
            #         self.get_logger().info(f'Achieved at mid point , obstacle ahead')
            #         self.forward_obstract_distance[0] = "y"
            #         self.obstacle_found = True
            #     elif ( not self.y_achieved ) and round(self.vehicle_local_position.y,0) != 0.0:
            #         self.publish_position_setpoint("position", self.vehicle_local_position.x,self.vehicle_local_position.y-1, self.takeoff_height,self.yaw_angle)
            #     elif ( not self.y_achieved ) and round(self.vehicle_local_position.y,0) == 0.0 :
            #         self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
            #         self.square_check += 1
            #         self.y_achieved = True

                # self.publish_position_setpoint("position", 0.0,0.0, self.takeoff_height)
                # if round(self.vehicle_local_position.y,0) == 0.0 :
                #     self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                #     self.square_check += 1
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.y_achieved and self.x_achieved:
                self.land()
                exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


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