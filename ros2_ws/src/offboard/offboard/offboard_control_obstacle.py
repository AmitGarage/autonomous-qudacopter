#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, ObstacleDistance,VehicleAttitude
import time
import math


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
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,'/fmu/out/vehicle_attitude',self.attitude_callback,qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.base_distance_factor = 10
        self.takeoff_height = -5.0
        self.forward_distance_x = 5.0*self.base_distance_factor
        self.forward_distance_y = 5.0*self.base_distance_factor
        self.square_check = 0
        self.obstacle_distance = ObstacleDistance()
        self.obstacle_found = False
        self.obstract_distance_x = 0.0
        self.obstract_distance_y = 0.0
        self.obstract_distance_z = 0.0
        self.current_heading = 0.0
        self.z_achieved = False
        self.yaw_angle = 1.57079

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def attitude_callback(self, msg):
        quaternion = [msg.q[0],msg.q[1],msg.q[2],msg.q[3]]

        self.current_heading = self.euler_from_quaternion(quaternion)
        self.get_logger().info(f'Current Heading: {math.degrees(self.current_heading):.2f} degrees')
    
    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        # self.get_logger().info(f'vehicle_local_position : {self.vehicle_local_position}') 

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
    
    def obstacle_distance_callback(self, msg):
        """
        This function is called when new data is received on the topic.
        """
        # self.get_logger().info(f'Obstacle distance: {msg.distances}')  # Print the distance
        # Add your code here to process the obstacle distance data
        all_angles_distance = msg.distances
        min_distance = min(msg.distances)
        self.get_logger().info(f'Min Obstacle distance: {min_distance}') 
        if (min_distance < 400) :
            self.obstacle_found = True
            self.get_logger().info(f'Current velocity : {self.vehicle_local_position}')
            self.get_logger().info(f'Obstacle Found : {all_angles_distance}')
            self.obstract_distance_x = self.vehicle_local_position.x - 2
            self.obstract_distance_y = self.vehicle_local_position.y - 2
            self.obstract_distance_z = self.vehicle_local_position.z
            # self.publish_offboard_control_heartbeat_signal("velocity")
            # self.publish_position_setpoint("velocity", self.vehicle_local_position.x-1,self.vehicle_local_position.y-1,self.vehicle_local_position.z-1,0.0 ,0.0, self.vehicle_local_position.vz)
            self.publish_offboard_control_heartbeat_signal("position")
            self.publish_position_setpoint("position", self.obstract_distance_x,self.obstract_distance_y,self.obstract_distance_z)
            # self.get_logger().info(f'Current position : {self.vehicle_local_position.x} , {self.vehicle_local_position.y} , {self.vehicle_local_position.z}')
            # time.sleep(30)

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
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position}")
                    self.publish_offboard_control_heartbeat_signal("rotate")
                    self.yaw_angle -= 1.57079
                    self.publish_position_setpoint("rotate", 0.0, 0.0, self.takeoff_height,self.yaw_angle)
                    self.z_achieved = True
                    self.yaw_speed += 1
                elif self.z_achieved and round(self.current_heading,3) != (self.yaw_angle * 180 )/ (3.14):
                    self.get_logger().info(f"rotating - {self.vehicle_local_position}")
                    self.publish_offboard_control_heartbeat_signal("rotate")
                    self.publish_position_setpoint("rotate", 0.0, 0.0, self.takeoff_height,self.yaw_angle)
                    self.yaw_speed += 1
                elif self.z_achieved and round(self.current_heading,3) == (self.yaw_angle * 180 )/ (3.14) :
                    self.square_check += 1
                    self.yaw_speed = 0
                    self.get_logger().info(f"{self.square_check} {self.current_heading} rotate completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    # time.sleep(30)

            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 1:
                
                self.offboard_setpoint_counter += 1
                if self.offboard_setpoint_counter > 20 :
                    self.get_logger().info(f"{self.offboard_setpoint_counter} - slowing - {self.vehicle_local_position}")
                    # self.get_logger().info(f'Obstacle Found , going in sleep mode for 30 seconds : {all_angles_distance}')
                    self.publish_offboard_control_heartbeat_signal("velocity")
                    self.publish_position_setpoint("velocity", self.vehicle_local_position.x,self.vehicle_local_position.y,self.vehicle_local_position.z,0.0 ,0.0, 1.0)
                else :
                    self.publish_position_setpoint("position", self.forward_distance_x, 0.0, self.takeoff_height)
                if round(self.vehicle_local_position.x,0) == self.forward_distance_x :
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.square_check += 1
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 2:
                self.publish_position_setpoint("position", self.forward_distance_x, self.forward_distance_y, self.takeoff_height)
                if round(self.vehicle_local_position.y,0) == self.forward_distance_y :
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.square_check += 1
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 3:
                self.publish_position_setpoint("position", 0.0, self.forward_distance_y, self.takeoff_height)
                if round(self.vehicle_local_position.x,0) == 0.0 :
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.square_check += 1
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 4:
                self.publish_position_setpoint("position", 0.0,0.0, self.takeoff_height)
                if round(self.vehicle_local_position.y,0) == 0.0 :
                    self.get_logger().info(f"{self.square_check} square completed - {self.vehicle_local_position.x} - {self.vehicle_local_position.y} - {self.vehicle_local_position.z}")
                    self.square_check += 1
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.square_check == 5:
                self.land()
                exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)