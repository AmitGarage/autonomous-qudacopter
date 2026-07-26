#!/usr/bin/env python3

"""
offboard_control.py
-------------------
ROS2 node for autonomous PX4 drone navigation in offboard mode.

Two operating modes
===================

Mode 1 🟩 WAYPOINT (user-directed)
    Drone receives (x, y, z) waypoints via /traverse_coordinates_topic.
    For each waypoint:
        1. Rotate yaw to face X target.
        2. Fly to X target (obstacle avoidance may produce detour waypoints).
        3. Rotate yaw to face Y target.
        4. Fly to Y target.
    Obstacle avoidance runs continuously via the LiDAR callback.
    If new coordinates arrive at any point, they are queued; the current
    navigation step is completed cleanly before the next one starts.

Mode 2 🟦 AUTONOMOUS (frontier-based house exploration)
    Drone independently explores the environment using a 2-D occupancy grid
    built from LiDAR data. The algorithm:
        1. LiDAR continuously marks cells as FREE or OCCUPIED.
        2. FRONTIER cells (FREE cells adjacent to unexplored space) are found.
        3. The nearest visible frontier becomes the next navigation target.
        4. Navigation uses the same waypoint-following + obstacle-avoidance
            logic as Mode 1; the drone dodges walls automatically.
        5. When no frontiers remain, the drone lands.
    If the user injects a waypoint at any time, the drone immediately
    switches to Mode 1 for that waypoint, then resumes autonomous exploration.

Structured log format (written to file logger; for post-flight plotting)
========================================================================
    process="event name"
    curr_x=<m> curr_y=<m> curr_z=<m> curr_yaw_rad=<rad> curr_yaw_deg=<deg>
    des_x=<m> des_y=<m> des_z=<m> des_yaw_rad=<rad>
    diff_x=<m> diff_y=<m> diff_z=<m> diff_yaw_rad=<rad>
    intern_x=<m> intern_y=<m>
    obstacle_found=<bool> obs_tgt_x=<m> obs_tgt_y=<m>
    lidar_front=<m> lidar_back=<m> lidar_left=<m> lidar_right=<m>
    lidar_front_right=<m> lidar_rear_right=<m> lidar_rear_left=<m> lidar_front_left=<m>dir_sign_x=<+1|-1> dir_sign_y=<+1|-1>
    [additional key-value pairs per event]
"""

import math
import os
import queue
import shutil
import datetime
import json
import logging
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition, VehicleStatus,
)

from sensor_msgs.msg import LaserScan

from utils.convert_log_file import convert
from custom_msgs.msg import TraverseCoordinates


class DroneMode(Enum):
    WAYPOINT   = "WAYPOINT"    # Mode 1: navigate to user-specified waypoints
    AUTONOMOUS = "AUTONOMOUS"  # Mode 2: frontier-based autonomous house exploration


class OffboardControl(Node):
    """
    ROS2 node that controls a PX4 drone in offboard mode.

    Subuscribes to:
    ---------------
    /fmu/out/vehicle_local_position_v1  VehicleLocalPosition
    /fmu/out/vehicle_status_v1          VehicleStatus
    <lidar_topic_name>                  LaserScan
    /traverse_coordinates_topic         TraverseCoordinates

    Publishes to:
    -------------
    /fmu/in/offboard_control_mode       OffboardControlMode (100 Hz heartbeat)
    /fmu/in/trajectory_setpoint         TrajectorySetpoint
    /fmu/in/vehicle_command             VehicleCommand
    """

    def __init__(self) -> None:
        super().__init__('offboard_control')

        # -- ROS Parameters -------------------------------------------------------------------------
        self.declare_parameter('lidar_topic_name', '/scan')
        self.declare_parameter('static_log_file_name', '')
        self.declare_parameter('lidar_rotation_anticlockwise_direction', False)
        self.declare_parameter('lidar_angle_resolution_in_degree', 0.5)
        self.declare_parameter('only_takeoff_and_land', False)
        self.declare_parameter('lidar_processed_file_name', '')
        # Mode 2 parameters
        self.declare_parameter('enable_autonomous_exploration', False)
        self.declare_parameter('default_takeoff_height', -1.5)     # NED z (negative = up)
        self.declare_parameter('grid_resolution_m', 0.5)             # meters per grid cell
        self.declare_parameter('frontier_min_distance', 0.8)       # ignore frontiers closer than this (m)

        lidar_topic_name                      = self.get_parameter('lidar_topic_name').get_parameter_value().string_value
        self.log_file_name                    = self.get_parameter('static_log_file_name').get_parameter_value().string_value
        self.lidar_processed_file_name        = self.get_parameter('lidar_processed_file_name').get_parameter_value().string_value
        self.only_takeoff_and_land            = self.get_parameter('only_takeoff_and_land').get_parameter_value().bool_value
        self.lidar_direction_reverse          = self.get_parameter('lidar_rotation_anticlockwise_direction').get_parameter_value().bool_value
        self.lidar_angle_resolution_in_degree = self.get_parameter('lidar_angle_resolution_in_degree').get_parameter_value().double_value
        enable_auto                           = self.get_parameter('enable_autonomous_exploration').get_parameter_value().bool_value
        self.deffault_takeoff_height          = self.get_parameter('default_takeoff_height').get_parameter_value().double_value
        self.grid_resolution                  = self.get_parameter('grid_resolution_m').get_parameter_value().double_value
        self.frontier_min_distance            = self.get_parameter('frontier_min_distance').get_parameter_value().double_value

        # Replace 'start_time' placeholder with current timestamp
        now       = datetime.datetime.now()
        timestamp = (
            str(now.year).zfill(4) + str(now.month).zfill(2) + str(now.day).zfill(2)
            + str(now.hour).zfill(2) + str(now.minute).zfill(2) + str(now.second).zfill(2)
        )
        self.log_file_name = self.log_file_name.replace('start_time', timestamp)
        # self._log_event(f"LOG_FILE_NAME: {self.log_file_name}")

        # -- File logger (post-flight analysis / plotting) ----------------------------------------------------------------
        self.logger = logging.getLogger('offboard_control_logger')
        self.logger.setLevel(logging.DEBUG)
        _fh = logging.FileHandler(self.log_file_name)
        _fh.setLevel(logging.DEBUG)
        _fh.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        self.logger.addHandler(_fh)
        self.logger.propagate = False

        # self._log_event("NODE_INITALIZED")

        # -- QoS Profiles for PX4 topics ----------------------------------------------------------------------------------

        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -- Publishers ---------------------------------------------------------------------------------------------------
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', qos_profile_pub)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_profile_pub)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', qos_profile_pub)

        # -- Subscribers ----------------------------------------------------------------------------------------------------
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile_sub)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile_sub)
        self.lidar_2d_subscription = self.create_subscription(LaserScan, lidar_topic_name, self.obstacle_distance_callback, 10)
        self.traverse_coordinates_subscriber = self.create_subscription(TraverseCoordinates, '/traverse_coordinates_topic', self.traverse_coordinates_callback, 10)

        # -- Drone state ( populated by callbacks ) -------------------------------------------------------------------------
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # -- Operation mode -------------------------------------------------------------------------------------------------
        self.drone_mode = DroneMode.AUTONOMOUS if enable_auto else DroneMode.WAYPOINT
        # When a user waypoint interrupts AUTONOMOUS mode, this flag allows resumption
        self.return_to_autonomous_after_waypoints = False

        # -- Mission phase flags --------------------------------------------------------------------------------------------
        # These encode the sub-state within the current navigation segment:
        #   x_rotate_achieved -> x_achieved -> y_rotate_achieved -> y_achieved
        # When BOTH x_achieved and y_achieved are True, the segment is done.
        self.offboard_setpoint_counter = 0     # Counts up to 11 before arming
        self.z_achieved                = False # True once takeoff altitude reached
        self.x_achieved                = True  # True = no X navigation pending
        self.y_achieved                = True  # True = no Y navigation pending
        self.x_rotate_achieved         = True  # True = yaw aligned toward X target
        self.y_rotate_achieved         = True  # True = yaw aligned toward Y target
        self.obstacle_found            = False # True while obstacle avoidancce is active
        self.armed                     = False

        # -- Waypoint targets ------------------------------------------------------------------------------------------------
        self.takeoff_height    = self.deffault_takeoff_height # NED z (negative = up)
        self.forward_distance_x = 0.0                         # Desired NED X position (m)
        self.forward_distance_y = 0.0                         # Desired NED Y position (m)

        # Waypoint queue (Mode 1: loaded from /traverse_coordinates_topic)
        self.traverse_coordinates_queue = queue.Queue()

        # -- Obstacle-avoidance intermittent waypoint -------------------------------------------------------------------------
        # [0] = active-traversal axis ("x" or "y")
        # [1] = dodge/stop target X (0.0 = not active)
        # [2] = dodge/stop target Y (0.0 = not active)
        self.forward_obstacle_target = ["x", 0.0, 0.0]

        # Lateral dodge direction and magnitude [x_component, y_component]
        self.continue_direction = [0.0, 0.0]

        # -- Incremental setpoints sent to PX4 each timer tick ---------------------------------------------
        self.intermittent_distance_x = 0.0
        self.intermittent_distance_y = 0.0
        self.intermittent_yaw        = 0.0

        # Minimum safe clearance from drone body (m); drives collision zone checks
        self.safe_distance_from_quadcopter = 0.6

        # -- Yaw / heading tracking -------------------------------------------------------------------
        self.yaw_angle               = math.pi         # Desired yaw setpoint (rad, [-pi,pi])
        self.start_angle             = math.pi         # Initial heading captured at arming
        self.actual_angle            = self.yaw_angle
        self.actual_angle_difference = 0.0
        self.start_angle_initialized = False

        # Movement direction indicators: True = drone needs positive-axis travel
        self.drone_current_direction      = [True,True]
        self.drone_current_direction_sign = [+1,+1]

        # Previous front-obstacle count for avoidance-stall detection
        self.previous_front_obstacle_found = 0.0

        # -- Latest LiDAR sector minimums (m) -------------------------------------------------------
        # Cached after each scan; '' until first scan arrives so log show empty
        self.lidar_front       = ''
        self.lidar_back        = ''
        self.lidar_left        = ''
        self.lidar_right       = ''
        self.lidar_front_right = ''
        self.lidar_front_left  = ''
        self.lidar_rear_right  = ''
        self.lidar_rear_left   = ''

        # -- LiDAR point-colud map (plotting frame; exported to JSON) -------------------------------
        self.x_data       = []  # Obstacle absolute X (plot frame, = -ned_x)
        self.y_data       = []  # Obstacle absolute Y (= ned_y)
        self.drone_x_data = []  # Drone track X (= ned_x)
        self.drone_y_data = []  # Drone track Y (= ned_y)

        # -- Occupancy grid (Mode 2) ----------------------------------------------------------------
        # Key: (gx, gy) integer grid cell in NED frame (gx * resolution = ned_x).
        # Value: 'free' or 'occupied'.  Absent key = unknown.
        self.occupancy_grid = {}
        # Set of (gx, gy) cells already targeted as frontiers (to avoid revisting)
        self.auto_visited_cells = set()

        # Unused fields kept for extrnal reader compatibility
        self.vehicle_step_distance = 0.0
        self.obstruct_distance_x   = 0.0
        self.obstruct_distance_y   = 0.0
        self.obstruct_distance_z   = 0.0

        # 10 ms control loop
        self.timer = self.create_timer(0.1, self.timer_callback)

    # -----------------------------------------------
    # Structured log helper
    # -----------------------------------------------
    def _log_event(self, process: str, **extra) -> None:
        """
        Write a structured, fixed-schema log line to the file logger.

        Every line always contains the full drone state so any single line is
        sufficient to reconstruct what happened at that moment. Fields not yet
        available (e.g. LiDAR before first scan) are written as ''.

        Fixed columns
        --------------
        process          Human-readable description of the current activity
        mode             Current DroneMode (WAYPOINT / AUTONOMOUS)
        curr_x/y/z       Drone NED position (m)
        curr_yaw_rad/deg Drone heading
        des_x/y/z        Desired waypoint position (m)
        des_yaw_rad      Desired yaw setpoint (rad)
        diff_x/y/z       Position error = desired - current (m)
        diff_yaw_rad     Yaw error = desired - current, wrapped to [-π, π]
        intern_x/y       Current incremental target sent to PX4 (m)
        obstacle_found   Whether obstacle avoidance is active
        obs_tgt_x/y      Obstacle-avoidance waypoint (m); '' when inactive
        lidar_*          Minimum LiDAR distance sector (m); '' until first scan
        dir_sign_x/y     Movement direction signs (+1 or -1)
        Extra key=value pairs appended at end via **extra kwargs.
        """

        pos = self.vehicle_local_position
        cx = round(pos.x, 3)
        cy = round(pos.y, 3)
        cz = round(pos.z, 3)
        cyaw_rad = round(pos.heading, 3)
        cyaw_deg = round(math.degrees(pos.heading), 2)

        dx = self.forward_distance_x
        dy = self.forward_distance_y
        dz = self.takeoff_height
        dyaw = round(self.yaw_angle, 3)

        diff_x = round(dx - cx, 3)
        diff_y = round(dy - cy, 3)
        diff_z = round(dz - cz, 3)
        diff_yaw = round(self.wrap_angle(self.yaw_angle - pos.heading), 3)

        obs_tgt_x = round(self.forward_obstacle_target[1], 3)
        obs_tgt_y = round(self.forward_obstacle_target[2], 3)

        extra_str = ', '.join(f'{k}={v}' for k, v in extra.items()) if extra else ''

        line = (
            f'process="{process}" mode={self.drone_mode.value} '
            f'curr_x={cx} curr_y={cy} curr_z={cz} '
            f'curr_yaw_rad={cyaw_rad} curr_yaw_deg={cyaw_deg} '
            f'des_x={dx} des_y={dy} des_z={dz} des_yaw_rad={dyaw} '
            f'diff_x={diff_x} diff_y={diff_y} diff_z={diff_z} diff_yaw_rad={diff_yaw} '
            f'intern_x={round(self.intermittent_distance_x, 3)} '
            f'intern_y={round(self.intermittent_distance_y, 3)} '
            f'obstacle_found={self.obstacle_found} '
            f'obs_tgt_x={obs_tgt_x} obs_tgt_y={obs_tgt_y} '
            f'lidar_front={self.lidar_front} lidar_back={self.lidar_back} '
            f'lidar_left={self.lidar_left} lidar_right={self.lidar_right} '
            f'lidar_front_right={self.lidar_front_right} '
            f'lidar_front_left={self.lidar_rear_right} '
            f'lidar_rear_right={self.lidar_rear_left} '
            f'lidar_rear_left={self.lidar_front_left} '
            f'dir_sign_x={self.drone_current_direction_sign[0]} '
            f'dir_sign_y={self.drone_current_direction_sign[1]}'
        )
        if extra_str:
            line += f' {extra_str}'
        self.logger.info(line)

    # ------------------------------------------------------------
    # Subscriber callbacks
    # -------------------------------------------------------------

    def traverse_coordinates_callback(self, traverse_coordinates_msg):
        """
        Receive user-specified waypoints and enqueue them.

        If in AUTONOMOUS mode at the time a message arrives:
            - The drone immediately switches to WAYPOINT mode and begins
              navigating to the first injected coordinate.
            - After all injected waypoints are consumed, the drone returns
              to AUTONOMOUS exploration automatically.

        While in WAYPOINT mode, new messages simply append to the queue.
        """
        self._log_event(f"NEW_WAYPOINT_RECEIVED: {traverse_coordinates_msg}")
        for row_no in range(traverse_coordinates_msg.rows):
            start = row_no * traverse_coordinates_msg.cols
            end   = start + traverse_coordinates_msg.cols
            self.traverse_coordinates_queue.put(traverse_coordinates_msg.data[start:end])
        self.reset_file(traverse_coordinates_msg.file_name)

        # If currently exploring autonomously, hand control to the user immediately
        if self.drone_mode == DroneMode.AUTONOMOUS and self.z_achieved:
            self.return_to_autonomous_after_waypoints = True
            self.drone_mode = DroneMode.WAYPOINT
            self._load_next_waypoint() # override current autonomous target now
            self._log_event("USER_WAYPOINT_INTERRUPTS_AUTONOMOUS_MODE")

    def reset_file(self, json_file_name):
        """Overwrite the waypoint JSON source file with an empty list after reading."""
        try:
            with open(json_file_name, 'w') as f:
                json.dump([], f)
            self._log_event("WAYPOINT_RESET_TO_EMPTY")
        except Exception as e:
            self._log_event(f"FAILED_TO_RESET_WAYPOINT: {e}")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Cache the latest local position and update movement direction signs."""
        self.vehicle_local_position = vehicle_local_position
        self.drone_x_data.append(-1 * vehicle_local_position.x)
        self.drone_y_data.append(vehicle_local_position.y)

        if self.forward_distance_x - vehicle_local_position.x >= 0:
            self.drone_current_direction[0] = True
            self.drone_current_direction_sign[0] = +1
        else:
            self.drone_current_direction[0] = False
            self.drone_current_direction_sign[0] = -1

        if self.forward_distance_y - vehicle_local_position.y >= 0:
            self.drone_current_direction[1] = True
            self.drone_current_direction_sign[1] = +1
        else:
            self.drone_current_direction[1] = False
            self.drone_current_direction_sign[1] = -1

    def vehicle_status_callback(self, vehicle_status):
        """Cache the latest vehicle status."""
        self.vehicle_status = vehicle_status

    # ---------------------------------------------------------------------------
    # LiDAR processing
    # ---------------------------------------------------------------------------

    def lidar_processing(self, lidar_msg):
        """
        Project LiDAR rays to absolute world-frame obstacle coordinates and
        accumulate them in x_data / y_data for point-cloud visualisation.

        Writes the map atomically (tmp + final) so the visualiser always sees
        a complete snapshot. Skips during takeoff or non-traversal phases.
        """
        ranges = lidar_msg.ranges.tolist()
        if self.lidar_direction_reverse:
            ranges = ranges[::-1]

        active_traversal = (
            self.z_achieved
            and (
                (not self.x_achieved and self.x_rotate_achieved)
                or (not self.y_achieved and self.y_rotate_achieved)
                or self.only_takeoff_and_land
            )
        )

        if not active_traversal:
            return

        obstacle_distances = [(idx, dist) for idx, dist in enumerate(ranges) if dist < np.inf]

        for idx, dist in obstacle_distances:
            angle_rad = (
                math.radians(180)
                - self.actual_angle_difference
                - self.vehicle_local_position.heading
                + math.radians(idx * self.lidar_angle_resolution_in_degree)
            )
            rel_x = dist * np.cos(angle_rad)
            rel_y = dist * np.sin(angle_rad)
            self.x_data.append(rel_x + (-1 * self.vehicle_local_position.x))
            self.y_data.append(rel_y + self.vehicle_local_position.y)

        self._export_map_data()

    def _export_map_data(self):
        """
        Atomically write the accumulated map data (point cloud + occupancy grid)
        to the configured JSON file for live visualisation.
        """

        # Build grid export: free/occupied cells in NED coordinates
        free_cells = []
        occupied_cells = []
        for (gx, gy), state in self.occupancy_grid.items():
            wx, wy = self._grid_to_ned(gx, gy)
            if state == 'free':
                free_cells.append([wx, wy])
            else:
                occupied_cells.append([wx, wy])

        map_data = {
            "x": self.x_data,
            "y": self.y_data,
            "drone_x": self.drone_x_data,
            "drone_y": self.drone_y_data,
            "grid_free": free_cells,
            "grid_occupied": occupied_cells,
        }
        tmp_path = self.lidar_processed_file_name.replace(".json", "_tmp.json")
        final_path = self.lidar_processed_file_name
        if os.path.isfile(tmp_path):
            shutil.copy(tmp_path, final_path)
        with open(tmp_path, "w") as f:
            json.dump(map_data, f, indent=4)

    def collision_detected(self, distances):
        """
        Emergency land if any LiDAR ray is within 0.3 m (hard collision threshold).
        """
        if any(d <= 0.3 for d in distances):
            self._log_event(
                'COLLISION IMMINENT: nearest obstacle <= 0.3 m. Commanding land!'
            )
            self._log_event('COLLISION_IMMINENT', collision_threshold_m=0.3)
            self.land()
            convert(self.log_file_name)
            exit()

    def obstacle_distance_callback(self, msg):
        """
        Main LiDAR callback executed for every scan.

        Tasks:
        1. Update occupancy grid for autonomous mapping (both modes).
        2. Update the visualisation point cloud.
        3. Check for imminent collision.
        4. Classify obstacles in 8 sectors.
        5. Compute dodge waypoint if a new obstacle is encountered.
        6. Run active avoidance manoeuvres if obstacle_found is True.
        """
        all_angles_distance = msg.ranges.tolist()
        if self.lidar_direction_reverse:
            all_angles_distance = all_angles_distance[::-1]

        # Update occupancy grid (used y Mode2; harmless in Mode1)
        self._update_occupancy_grid_from_lidar(all_angles_distance)

        self._log_event("LIDAR_SCAN_RAW", all_distances=all_angles_distance)

        self.lidar_processing(msg)
        self.collision_detected(all_angles_distance)

        # Sector classification at 3 m (avoidance) and 4 m (early warning)
        (back_cnt, left_cnt, front_cnt, right_cnt,
         left_min, right_min, back_min, front_min,
         front_right_min, rear_right_min, rear_left_min, front_left_min,
         front_left_cnt, front_right_cnt,
         front_left_dist_min, front_right_dist_min
         ) = self.obstacle_and_direction(msg, 3)

        (_, _, front_cnt_4m, back_cnt_4m, *_) = self.obstacle_and_direction(msg, 4)

        no_avoidance_active = not self.obstacle_found
        traversing_x = not self.x_achieved and self.forward_obstacle_target[1] == 0.0
        traversing_y = not self.y_achieved and self.forward_obstacle_target[2] == 0.0
        obstacle_incoming = (front_cnt_4m >= 2) or (back_cnt_4m >= 2)

        if no_avoidance_active and (traversing_x or traversing_y) and obstacle_incoming:
            self._compute_dodge_waypoint(front_min, back_min, right_min, left_min)
            self._log_event(
                "OBSTACLE DETECTED WHILE TRAVERSING",
                back_cnt=back_cnt, left_cnt=left_cnt,
                front_cnt=front_cnt, right_cnt=right_cnt,
                obs_tgt_x_set=round(self.forward_obstacle_target[1], 3),
                obs_tgt_y_set=round(self.forward_obstacle_target[2], 3),
                x_achieved=self.x_achieved, y_achieved=self.y_achieved
            )

        elif self.obstacle_found:
            self._run_obstacle_avoidance(
                back_cnt, left_cnt, front_cnt, right_cnt,
                front_left_cnt, front_right_cnt, 
                left_min, right_min, back_min, front_min,
                front_right_min, rear_right_min, rear_left_min, front_left_min,
                front_left_dist_min, front_right_dist_min,
                all_angles_distance
            )

        self.previous_front_obstacle_found = front_cnt

    # -----------------------------------------------------------------------
    # Occupancy grid helpers (Mode 2 - autonomous exploration)
    # ----------------------------------------------------------------------

    def _ned_to_grid(self, ned_x: float, ned_y: float) -> tuple:
        """Convert NED position (m) to integer grid cell (gx, gy)."""
        return (int(round(ned_x / self.grid_resolution)),
                int(round(ned_y / self.grid_resolution)))

    def _grid_to_ned(self, gx: int, gy: int) -> tuple:
        """Convert grid cell (gx, gy) to NED position (m) at cell centre."""
        return (gx * self.grid_resolution, gy * self.grid_resolution)

    def _update_occupancy_grid_from_lidar(self, all_ranges: list) -> None:
        """
        Update the 2-D occupancy grid from the latest LiDAR scan.

        For each ray:
        - Cells along the ray up to the obstacle are marked FREE.
        - The cell where the obstacle is detected is marked OCCUPIED.

        Uses the same coordinate formula as lidar_processing so the grid 
        is consistent with the point-colud map.

        Formula derivation (NED frame, heading h, LiDAR index i):
            angle = pi - actual_angle_difference - h - i * resolution_rad
            obstacle_ned_x = drone_x - d * cos(angle)
            obstacle_ned_y = drone_y + d * sin(angle)
        """
        if not self.z_achieved:
            return  # Don't map before the drone has reached operating altitude

        h = self.vehicle_local_position.heading
        cx = self.vehicle_local_position.x
        cy = self.vehicle_local_position.y
        res_rad = math.radians(self.lidar_angle_resolution_in_degree)

        # Always mark the drone's current cell as free
        self.occupancy_grid[self._ned_to_grid(cx, cy)] = 'free'

        for i, raw_dist in enumerate(all_ranges):
            angle = (
                math.radians(180)
                + self.actual_angle_difference
                + h
                + i * res_rad
            )

            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            # How far along this ray to mark free cells
            max_free_dist = min(raw_dist if raw_dist < np.inf else 8.0, 8.0)
            steps = max(int(max_free_dist / self.grid_resolution), 1)

            for step in range(1, steps + 1):
                d = step * self.grid_resolution
                ned_x = cx - d * cos_a
                ned_y = cy + d * sin_a
                cell = self._ned_to_grid(ned_x, ned_y)
                if self.occupancy_grid.get(cell) != 'occupied':
                    self.occupancy_grid[cell] = 'free'

            # Mark the obstacle cell
            if raw_dist < np.inf:
                ned_x = cx - raw_dist * cos_a
                ned_y = cy + raw_dist * sin_a
                self.occupancy_grid[self._ned_to_grid(ned_x, ned_y)] = 'occupied'

    def _find_nearest_frontier(self):
        """
        Find the nearest unvisited frontier cell.

        A FRONTIER is a FREE cell that has at least one neighbour with an
        unknown (unmapped) state. Frontiers are the boundaries between what
        the drone has seen and what it hasn't yet explored.

        Returns:
            (ned_x, ned_y) of the nearest frontier, or None if none exist.
        """
        cx = self.vehicle_local_position.x
        cy = self.vehicle_local_position.y

        best_dist = float('inf')
        best_frontier = None

        for (gx, gy), state in list(self.occupancy_grid.items()):
            if state != 'free':
                continue
            if (gx, gy) in self.auto_visited_cells:
                continue  # already targeted

            # Check if any cardinal neighbour is unexplored (not in grid)
            has_unknown_neighbour = any(
                (gx + dx, gy + dy) not in self.occupancy_grid
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]
            )
            if not has_unknown_neighbour:
                continue  # not a frontier

            fx, fy = self._grid_to_ned(gx, gy)
            dist = math.sqrt((fx - cx) ** 2 + (fy - cy) ** 2)

            # Skip frontiers that are too close (likely noise / already scanned)
            if dist < self.frontier_min_distance:
                continue

            if dist < best_dist:
                best_dist = dist
                best_frontier = (fx, fy, gx, gy)

        if best_frontier is None:
            return None

        fx, fy, gx, gy = best_frontier
        # Pre-mark as visited so concurrent calls don't target the same cell
        self.auto_visited_cells.add((gx, gy))
        return fx, fy

    # ------------------------------------------------------------------------
    # Obstacle avoidance state machine (unchanged from previous version)
    # ------------------------------------------------------------------------

    def _compute_dodge_waypoint(self, front_min, back_min, right_min, left_min):
        """
        Compute the intermediate stop/back-off waypoint when an obstacle
        is detected ahead during traversal and store it in forward_obstacle_target.
        """

        sdq = self.safe_distance_from_quadcopter

        if (front_min - (sdq + sdq / 2)) < sdq:
            mid_distance = abs(front_min - sdq) + 0.25
        elif (back_min - (sdq + sdq / 2)) < sdq:
            mid_distance = abs(back_min - sdq) + 0.25
        elif (front_min - (sdq + sdq / 2)) > 0.0:
            mid_distance = -1 * (front_min - (sdq + sdq / 2))
        else:
            mid_distance = -1

        if not self.x_achieved:
            self.forward_obstacle_target[1] = (
                self.vehicle_local_position.x
                - (self.drone_current_direction_sign[0] * mid_distance)
            )
            if (mid_distance < 0
                    and (self.forward_obstacle_target[1] - self.forward_distance_x)
                    * self.drone_current_direction_sign[0] > 0):
                self.forward_obstacle_target[1] = self.forward_distance_x
            if (mid_distance > 0
                    and (self.forward_obstacle_target[1] - self.forward_distance_x)
                    * self.drone_current_direction_sign[0] < 0):
                x_gap = abs(self.vehicle_local_position.x - self.forward_distance_x)
                if (front_min - x_gap) > sdq:
                    self.forward_obstacle_target[1] = (
                        self.vehicle_local_position.x
                        + self.drone_current_direction_sign[0] * x_gap
                    )

            if right_min < sdq * 2 and left_min > sdq * 2:
                self.forward_obstacle_target[2] = (
                    self.vehicle_local_position.y + (-0.25 * self.drone_current_direction_sign[0]))
            elif right_min > sdq * 2 and left_min < sdq * 2:
                self.forward_obstacle_target[2] = (
                    self.vehicle_local_position.y + (0.25 * self.drone_current_direction_sign[0]))

        if not self.y_achieved:
            self.forward_obstacle_target[2] = (
                self.vehicle_local_position.y
                - (self.drone_current_direction_sign[1] * mid_distance)
            )
            if (mid_distance < 0
                    and (self.forward_obstacle_target[2] - self.forward_distance_y)
                    * self.drone_current_direction_sign[1] > 0):
                self.forward_obstacle_target[2] = self.forward_distance_y
            if (mid_distance > 0
                    and (self.forward_obstacle_target[2] - self.forward_distance_y)
                    * self.drone_current_direction_sign[1] < 0):
                y_gap = abs(self.vehicle_local_position.y - self.forward_distance_y)
                if (front_min - y_gap) > sdq:
                    self.forward_obstacle_target[2] = (
                        self.vehicle_local_position.y
                        + self.drone_current_direction_sign[1] * y_gap
                    )

            if right_min < sdq * 2 and left_min > sdq * 2:
                self.forward_obstacle_target[1] = (
                    self.vehicle_local_position.x + (0.25 * self.drone_current_direction_sign[1]))
            elif right_min > sdq * 2 and left_min < sdq * 2:
                self.forward_obstacle_target[1] = (
                    self.vehicle_local_position.x + (-0.25 * self.drone_current_direction_sign[1]))

    def _run_obstacle_avoidance(
        self,
        back_cnt, left_cnt, front_cnt, right_cnt,
        front_left_cnt, front_right_cnt,
        left_min, right_min, back_min, front_min,
        front_right_min, rear_right_min, rear_left_min, front_left_min,
        front_left_dist_min, front_right_dist_min,
        all_angles_distance
    ):
        """Dispatch to X-axis or Y-axis avoidance depending on which axis was active."""
        if self.forward_obstacle_target[0] == "x":
            self._obstacle_avoidance_x_axis(
                back_cnt, left_cnt, front_cnt, right_cnt,
                front_left_cnt, front_right_cnt,
                left_min, right_min, back_min, front_min,
                front_right_min, rear_right_min, rear_left_min, front_left_min,
                front_left_dist_min, front_right_dist_min, all_angles_distance
            )
        elif self.forward_obstacle_target[0] == "y":
            self._obstacle_avoidance_y_axis(
                back_cnt, left_cnt, front_cnt, right_cnt,
                front_left_cnt, front_right_cnt,
                left_min, right_min, back_min, front_min,
                front_right_min, rear_right_min, rear_left_min, front_left_min,
                front_left_dist_min, front_right_dist_min, all_angles_distance
            )

    def _obstacle_avoidance_x_axis(
        self,
        back_cnt, left_cnt, front_cnt, right_cnt,
        front_left_cnt, front_right_cnt,
        left_min, right_min, back_min, front_min,
        front_right_min, rear_right_min, rear_left_min, front_left_min,
        front_left_dist_min, front_right_dist_min,
        all_angles_distance
    ):
        """
        Lateral dodge along Y when obstacle encountered during X-axis traversal.

        Priority:
          A. Continue previous dodge if front still partially blocked.
          B-C. Choose cleaner side (left / right) based on front-sector counts.
          D. Both sides blocked: choose the side with fewer obstacles.
          E. All clear -> resume X traversal.
          F. Fallback: keep current dodge direction.
        """
        sdq = self.safe_distance_from_quadcopter
        self._log_event(
            "OBS_AVOID_X_ACTIVE",
            back_cnt=back_cnt, left_cnt=left_cnt,
            front_cnt=front_cnt, right_cnt=right_cnt,
            front_left_cnt=front_left_cnt, front_right_cnt=front_right_cnt
        )

        # A: Continue previous dodge
        if (front_cnt > 0
                and front_cnt <= self.previous_front_obstacle_found
                and self.continue_direction[1] != 0
                and left_min >= sdq and right_min >= sdq and back_min >= sdq):

            if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2) \
                    and abs(self.continue_direction[1]) == 1:
                self.continue_direction[1] /= 4
            elif (left_min >= sdq * 2 and right_min >= sdq * 2
                    and back_min >= sdq * 2 and abs(self.continue_direction[1]) == 0.25):
                self.continue_direction[1] *= 4

            if (front_right_min < sdq * 2 and rear_right_min < sdq * 2
                    and left_min > sdq * 2):
                self.continue_direction[1] = -(0.25 * self.drone_current_direction_sign[0])
                self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_X_RIGHT_CORNERS_BLOCKED_SLIDE_LEFT",
                    dodge_y=round(self.continue_direction[1], 3))

            if (front_left_min < sdq * 2 and rear_left_min < sdq * 2
                    and right_min > sdq * 2):
                self.continue_direction[1] = (0.25 * self.drone_current_direction_sign[0])
                self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_X_LEFT_CORNERS_BLOCKED_SLIDE_RIGHT",
                    dodge_y=round(self.continue_direction[1], 3))

            elif (front_min > sdq * 2
                    and (rear_right_min < sdq * 2 or rear_left_min < sdq * 2)):
                self.intermittent_distance_x = self.vehicle_local_position.x + (0.25 * self.drone_current_direction_sign[0])
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_X_REAR_CORNERS_PUSH_FORWARD")

            elif (back_min > sdq * 2
                    and (front_right_min < sdq * 2 or front_left_min < sdq * 2)):
                self.intermittent_distance_x = self.vehicle_local_position.x - (0.25 * self.drone_current_direction_sign[0])
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_X_FRONT_CORNERS_PULL_BACKWARD")

            else:
                if np.sign(self.drone_current_direction_sign[0]) == np.sign(self.continue_direction[1]):
                    y_diff = abs(right_min - sdq - 0.1)
                else:
                    y_diff = abs(left_min - sdq - 0.1)
                if y_diff > 1:
                    y_diff = np.sign(y_diff)
                self.continue_direction[1] = y_diff * np.sign(self.continue_direction[1])
                self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_X_CONTINUE_PREVIOUS_DODGE",
                    dodge_y=round(self.continue_direction[1], 3))

        # B: Dodge left (right side clearer)
        elif front_cnt >= 2 and front_left_cnt < front_right_cnt:
            y_diff = abs(left_min - sdq - 0.1)
            if y_diff > 1: y_diff = np.sign(y_diff)
            self.continue_direction[1] = self.drone_current_direction_sign[0] * y_diff * (-1)
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_FRONT_BLOCKED_DODGE_LEFT",
                dodge_y=round(self.continue_direction[1], 3),
                front_left_cnt=front_left_cnt, front_right_cnt=front_right_cnt)

        # C: Dodge right (left side clearer)
        elif front_cnt >= 2 and front_left_cnt >= front_right_cnt:
            y_diff = abs(right_min - sdq - 0.1)
            if y_diff > 1: y_diff = np.sign(y_diff)
            self.continue_direction[1] = self.drone_current_direction_sign[0] * y_diff
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_FRONT_BLOCKED_DODGE_RIGHT",
                dodge_y=round(self.continue_direction[1], 3),
                front_left_cnt=front_left_cnt, front_right_cnt=front_right_cnt)

        # D: Front + both sides blocked, prefer left
        elif front_cnt >= 2 and left_cnt >= 2 and right_cnt >= 2 and left_cnt <= right_cnt:
            self.continue_direction[1] = (
                self.drone_current_direction_sign[0] * (-0.25)
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[0] * (-1.0)
            )
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_3SIDES_BLOCKED_GO_LEFT", dodge_y=round(self.continue_direction[1], 3))

        # D: Front + both sides blocked, prefer right
        elif front_cnt >= 2 and left_cnt >= 2 and right_cnt >= 2 and left_cnt >= right_cnt:
            self.continue_direction[1] = (
                self.drone_current_direction_sign[0] * 0.25
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[0] * 1.0
            )
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_3SIDES_BLOCKED_GO_RIGHT", dodge_y=round(self.continue_direction[1], 3))

        elif front_cnt >= 2 and left_cnt == right_cnt and self.continue_direction[1] != 0.0:
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_EQUAL_SIDES_CONTINUE_DODGE", dodge_y=round(self.continue_direction[1], 3))

        elif (left_cnt < 2 and (right_cnt >= 2 or left_cnt <= right_cnt) and front_cnt >= 2):
            self.continue_direction[1] = (
                self.drone_current_direction_sign[0] * (-0.25)
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[0] * (-1.0)
            )
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_RIGHT_BLOCKED_GO_LEFT", dodge_y=round(self.continue_direction[1], 3))

        elif (right_cnt < 2 and (left_cnt >= 2 or right_cnt <= left_cnt) and front_cnt >= 2):
            self.continue_direction[1] = (
                self.drone_current_direction_sign[0] * (-0.25)
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[0] * (1.0)
            )
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_LEFT_BLOCKED_GO_RIGHT", dodge_y=round(self.continue_direction[1], 3))

        elif front_cnt <= 1 and left_min >= sdq * 2 and right_min < sdq:
            self.continue_direction[1] = self.drone_current_direction_sign[0] * (-0.25)
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_RIGHTL_TOO_CLOSE_NUDGE_LEFT",
                right_min=right_min, dodge_y=round(self.continue_direction[1], 3))

        elif front_cnt <= 1 and left_min < sdq and right_min >= sdq * 2:
            self.continue_direction[1] = self.drone_current_direction_sign[0] * 0.25
            self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_X_LEFT_WALL_TOO_CLOSE_NUDGE_RIGHT",
                left_min=left_min, dodge_y=round(self.continue_direction[1], 3))

        # E: All clear -> resume X traversal
        elif front_cnt < 1 and left_min >= sdq and right_min >= sdq and back_min >= sdq:
            self._log_event("OBS_X_PATH_CLEAR_RESUMING_TRAVERSAL",
                left_min=left_min, right_min=right_min, back_min=back_min)
            self.obstacle_found = False
            self.forward_obstacle_target[1] = 0.0
            self.forward_obstacle_target[2] = 0.0
            self.continue_direction[0] = 0.0
            self.continue_direction[1] = 0.0
            self.intermittent_distance_x = self.vehicle_local_position.x
            self.intermittent_distance_y = self.vehicle_local_position.y

        # F: Fallback
        else:
            if self.continue_direction[1] != 0.0:
                self.intermittent_distance_y = self.vehicle_local_position.y + self.continue_direction[1]
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_X_FALLBACK_CONTINUE_DODGE", dodge_y=round(self.continue_direction[1], 3))
            else:
                self._log_event("OBS_X_FALLBACK_NO_DODGE_DIRECTION_SET")

    def _obstacle_avoidance_y_axis(
        self,
        back_cnt, left_cnt, front_cnt, right_cnt,
        front_left_cnt, front_right_cnt,
        left_min, right_min, back_min, front_min,
        front_right_min, rear_right_min, rear_left_min, front_left_min,
        front_left_dist_min, front_right_dist_min,
        all_angles_distance
    ):
        """
        Lateral dodge along X when obstacle encountered during Y-axis traversal.
        Priority order mirrors _obstacle_avoidance_x_axis with x/y roles swapped.
        """
        sdq = self.safe_distance_from_quadcopter
        self._log_event(
            "OBS_AVOID_Y_ACTIVE",
            back_cnt=back_cnt, left_cnt=left_cnt,
            front_cnt=front_cnt, right_cnt=right_cnt,
            front_left_cnt=front_left_cnt, front_right_cnt=front_right_cnt
        )

        if (front_cnt > 0
                and front_cnt <= self.previous_front_obstacle_found
                and self.continue_direction[0] != 0.0
                and left_min >= sdq and right_min >= sdq and back_min >= sdq):

            if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2) \
                    and abs(self.continue_direction[0]) == 1:
                self.continue_direction[0] /= 4
            elif (left_min >= sdq * 2 and right_min >= sdq * 2
                    and back_min >= sdq * 2 and abs(self.continue_direction[0]) == 0.25):
                self.continue_direction[0] *= 4

            if (front_right_min < sdq * 2 and front_left_min < sdq * 2 and back_min > sdq * 2):
                self.intermittent_distance_y = self.vehicle_local_position.y - (self.drone_current_direction_sign[1] * 0.25)
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_FRONT_CORNERS_PULL_BACKWARD")
            elif (rear_right_min < sdq * 2 and rear_left_min < sdq * 2 and front_min > sdq * 2):
                self.intermittent_distance_y = self.vehicle_local_position.y + (self.drone_current_direction_sign[1] * 0.25)
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_REAR_CORNERS_PUSH_FORWARD")
            elif (rear_right_min < sdq * 2 and front_right_min < sdq * 2 and left_min > sdq * 2):
                self.intermittent_distance_y = self.vehicle_local_position.y + (self.drone_current_direction_sign[1] * 0.25)
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_RIGHT_SIDE_CORNERS_SLIDE_LEFT")
            elif (rear_left_min < sdq * 2 and front_left_min < sdq * 2 and right_min > sdq * 2):
                self.intermittent_distance_y = self.vehicle_local_position.y - (self.drone_current_direction_sign[1] * 0.25)
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_LEFT_SIDE_CORNERS_SLIDE_RIGHT")
            elif (front_min > sdq * 2
                    and (rear_right_min < sdq * 2 or rear_left_min < sdq * 2)):
                self.intermittent_distance_y = self.vehicle_local_position.y + (self.drone_current_direction_sign[1] * 0.25)
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_REAR_CORNERS_PUSH_BACKWARD")
            elif (back_min > sdq * 2 and right_min > sdq * 2 
                  and rear_right_min < sdq * 2 and front_left_min < sdq * 2):
                self.continue_direction[0] = self.drone_current_direction_sign[1] * 0.25
                self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
                self.intermittent_distance_y = self.vehicle_local_position.y - (self.drone_current_direction_sign[1] * 0.25)
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_FRONT_LEFT_CORNER_DIAG_RIGHT_DOWN",
                    dodge_x=round(self.continue_direction[0], 3))
            elif (back_min > sdq * 2
                    and (front_right_min < sdq * 2 or front_left_min < sdq * 2)):
                self.intermittent_distance_y = self.vehicle_local_position.y - (self.drone_current_direction_sign[1] * 0.25)
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_FRONT_CORNERS_PULL_BACKWARD")
            else:
                if np.sign(self.drone_current_direction_sign[1]) == np.sign(self.continue_direction[0]):
                    x_diff = abs(left_min - sdq - 0.1)
                else:
                    x_diff = abs(right_min - sdq - 0.1)
                if x_diff > 1: x_diff = np.sign(x_diff)
                self.continue_direction[0] = x_diff * np.sign(self.continue_direction[0])
                self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_CONTINUE_PREVIOUS_DODGE", dodge_x=round(self.continue_direction[0], 3))

        elif front_cnt >= 2 and front_left_cnt < front_right_cnt:
            x_diff = abs(left_min - sdq - 0.1)
            if x_diff > 1: x_diff = np.sign(x_diff)
            self.continue_direction[0] = self.drone_current_direction_sign[1] * x_diff
            self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_FRONT_BLOCKED_DODGE_LEFT", dodge_x=round(self.continue_direction[0], 3))

        elif front_cnt >= 2 and front_right_cnt <= front_left_cnt:
            x_diff = abs(right_min - sdq - 0.1)
            if x_diff > 1: x_diff = np.sign(x_diff)
            self.continue_direction[0] = self.drone_current_direction_sign[1] * x_diff * (-1)
            self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_FRONT_BLOCKED_DODGE_RIGHT", dodge_x=round(self.continue_direction[0], 3))

        elif front_cnt >= 2 and left_cnt >= 2 and right_cnt >= 2 and left_cnt <= right_cnt:
            self.continue_direction[0] = (
                self.drone_current_direction_sign[1] * 0.25
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[1] * 1.0
            )
            self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_3SIDES_BLOCKED_GO_LEFT", dodge_x=round(self.continue_direction[0], 3))

        elif front_cnt >= 2 and left_cnt >= 2 and right_cnt >= 2 and left_cnt >= right_cnt:
            self.continue_direction[0] = (
                self.drone_current_direction_sign[1] * (-0.25)
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[1] * (-1.0)
            )
            self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_3SIDES_BLOCKED_GO_RIGHT", dodge_x=round(self.continue_direction[0], 3))

        elif front_cnt >= 2 and left_cnt == right_cnt and self.continue_direction[0] != 0.0:
            self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_EQUAL_SIDES_CONTINUE_DODGE", dodge_x=round(self.continue_direction[0], 3))

        elif (left_cnt < 2 and (right_cnt >= 2 or left_cnt <= right_cnt) and front_cnt >= 2):
            self.continue_direction[0] = (
                self.drone_current_direction_sign[1] * 0.25
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[1] * 1.0
            )
            self.intermittent_distance_y = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_RIGHT_BLOCKED_GO_LEFT", dodge_y=round(self.continue_direction[1], 3))

        elif (right_cnt < 2 and (left_cnt >= 2 or right_cnt <= left_cnt) and front_cnt >= 2):
            self.continue_direction[1] = (
                self.drone_current_direction_sign[1] * (-0.25)
                if (left_min < sdq * 2 or right_min < sdq * 2 or back_min < sdq * 2)
                else self.drone_current_direction_sign[1] * (-1.0)
            )
            self.intermittent_distance_y = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_LEFT_BLOCKED_GO_RIGHT", dodge_y=round(self.continue_direction[1], 3))

        elif front_cnt <= 1 and left_min >= sdq * 2 and right_min < sdq:
            self.continue_direction[0] = self.drone_current_direction_sign[1] * 0.25
            self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_RIGHT_WALL_TOO_CLOSE_NUDGE_LEFT", right_min=right_min)

        elif front_cnt <= 1 and left_min < sdq and right_min >= sdq * 2:
            self.continue_direction[0] = self.drone_current_direction_sign[1] * (-0.25)
            self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
            self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
            self._log_event("OBS_Y_LEFT_WALL_TOO_CLOSE_NUDGE_RIGHT", left_min=left_min)

        elif front_cnt < 1 and left_min >= sdq and right_min >= sdq and back_min >= sdq:
            self._log_event("OBS_Y_PATH_CLEAR_RESUMING_TRAVERSAL",
                left_min=left_min, right_min=right_min, back_min=back_min)
            self.obstacle_found = False
            self.forward_obstacle_target[1] = 0.0
            self.forward_obstacle_target[2] = 0.0
            self.continue_direction[0] = 0.0
            self.continue_direction[1] = 0.0
            self.intermittent_distance_x = self.vehicle_local_position.x
            self.intermittent_distance_y = self.vehicle_local_position.y
        else:
            if self.continue_direction[0] != 0.0:
                self.intermittent_distance_x = self.vehicle_local_position.x + self.continue_direction[0]
                self.publish_position_setpoint("position", self.intermittent_distance_x, self.intermittent_distance_y, self.takeoff_height, self.yaw_angle)
                self._log_event("OBS_Y_FALLBACK_CONTINUE_DODGE", dodge_x=round(self.continue_direction[0], 3))
            else:
                self._log_event("OBS_Y_FALLBACK_NO_DODGE_DIRECTION_SET")

    def obstacle_and_direction(self, msg, threshold):
        """
        Classify LiDAR readings into 8 sectors and return per-sector obstacle counts
        and minimum distances. Also caches sector minimums for structured logging.

        Sector layout (0deg = front, clockwise):
          Front:      -25deg to +25deg    Front-Right: 35deg to 55deg
          Right:       65deg to 115deg    Rear-Right:  125deg to 145deg
          Back:       155deg to 205deg    Rear-Left:   215deg to 235deg
          Left:       245deg to 295deg    Front-Left:  305deg to 325deg
        """
        ranges = msg.ranges.tolist()
        if self.lidar_direction_reverse:
            ranges = ranges[::-1]

        r = self.lidar_angle_resolution_in_degree

        dist_front       = ranges[-int(25 / r):] + ranges[:int(25 / r)]
        dist_right       = ranges[int(65 / r):int(115 / r)]
        dist_back        = ranges[int(155 / r):int(205 / r)]
        dist_left        = ranges[int(245 / r):int(295 / r)]
        dist_front_right = ranges[int(35 / r):int(55 / r)]
        dist_rear_right  = ranges[int(125 / r):int(145 / r)]
        dist_rear_left   = ranges[int(215 / r):int(235 / r)]
        dist_front_left  = ranges[int(305 / r):int(325 / r)]
        dist_fl_half      = ranges[-int(25 / r):]
        dist_fr_half      = ranges[:int(25 / r)]

        back_cnt        = len([x for x in dist_back if x < threshold])
        left_cnt         = len([x for x in dist_left if x < threshold])
        front_cnt        = len([x for x in dist_front if x < threshold])
        right_cnt        = len([x for x in dist_right if x < threshold])
        front_left_cnt   = len([x for x in dist_fl_half if x < threshold])
        front_right_cnt  = len([x for x in dist_fr_half if x < threshold])

        left_min        = min(dist_left)
        right_min        = min(dist_right)
        back_min         = min(dist_back)
        front_min        = min(dist_front)
        front_right_min  = min(dist_front_right)
        rear_right_min   = min(dist_rear_right)
        rear_left_min    = min(dist_rear_left)
        front_left_min   = min(dist_front_left)
        fl_half_min      = min(dist_fl_half)
        fr_half_min      = min(dist_fr_half)

        # Cache for _log_event
        self._lidar_front       = round(front_min,       3)
        self._lidar_back        = round(back_min,        3)
        self._lidar_left        = round(left_min,        3)
        self._lidar_right       = round(right_min,       3)
        self._lidar_front_right = round(front_right_min, 3)
        self._lidar_rear_right  = round(rear_right_min,  3)
        self._lidar_rear_left   = round(rear_left_min,   3)
        self._lidar_front_left  = round(front_left_min,  3)

        return (
            back_cnt, left_cnt, front_cnt, right_cnt,
            left_min, right_min, back_min, front_min,
            front_right_min, rear_right_min, rear_left_min, front_left_min,
            front_left_cnt, front_right_cnt, fl_half_min, fr_half_min
        )

    # ------------------------------------------------------------------------
    # Vehicle command helpers
    # ------------------------------------------------------------------------

    def arm(self):
        """Send MAVLink arm command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self._log_event('ARM_COMMAND_SENT')

    def disarm(self):
        """Send MAVLink disarm command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self._log_event('DISARM_COMMAND_SENT')

    def engage_offboard_mode(self):
        """Switch PX4 to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self._log_event("SWITCHING_TO_OFFBOARD_MODE")

    def land(self):
        """Command PX4 to land at the current XY position."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._log_event("SWITCHING_TO_LAND_MODE")

    def publish_offboard_control_heartbeat_signal(self, move_type="position"):
        """
        Publish OffboardControlMode heartbeat (must arrive at >2 Hz or PX4
        exits offboard mode). move_type selects active control channels.
        """
        msg = OffboardControlMode()
        if move_type == "position":
            msg.position = True; msg.velocity = False
            msg.acceleration = False; msg.attitude = False; msg.body_rate = False
        elif move_type == "velocity":
            msg.position = True; msg.velocity = True
            msg.acceleration = False; msg.attitude = False; msg.body_rate = False
        elif move_type == "acceleration":
            msg.position = False; msg.velocity = False
            msg.acceleration = True; msg.attitude = False; msg.body_rate = False
        elif move_type == "rotate":
            msg.position = True; msg.velocity = False
            msg.acceleration = False; msg.attitude = True; msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
        self._log_event("HERTBEAT_SIGNAL_SEND")

    def publish_position_setpoint(
        self,
        move_type: str,
        x: float, y: float, z: float,
        yaw_angle: float = 0.0,
        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0
    ):
        """
        Publish a TrajectorySetpoint to PX4.
        x, y, z are NED positions (m); z negative = up.
        yaw_angle is in radians [-pi, pi].
        """
        msg = TrajectorySetpoint()
        if move_type == "position":
            msg.position = [x, y, z]; msg.yaw = yaw_angle
        elif move_type == "velocity":
            msg.position = [x, y, z]; msg.velocity = [vx, vy, vz]; msg.yaw = yaw_angle
        elif move_type == "acceleration":
            msg.acceleration = [x, y, z]
        elif move_type == "rotate":
            msg.position = [x, y, z]; msg.yaw = yaw_angle
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a MAVLink VehicleCommand; extra params passed as param1..param7."""
        msg = VehicleCommand()
        msg.command         = command
        msg.param1          = params.get("param1", 0.0)
        msg.param2          = params.get("param2", 0.0)
        msg.param3          = params.get("param3", 0.0)
        msg.param4          = params.get("param4", 0.0)
        msg.param5          = params.get("param5", 0.0)
        msg.param6          = params.get("param6", 0.0)
        msg.param7          = params.get("param7", 0.0)
        msg.target_system    = 1; msg.target_component = 1
        msg.source_system    = 1; msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # ------------------------------------------------------------------------
    # Motion utility helpers
    # ------------------------------------------------------------------------

    def adjust_position(self, current: float, desired: float,
            max_step: float = 0.03, gain: float = 0.15) -> float:
        """
        Proportional step from current toward desired position.
        Step = gain * error, clamped to [0.5 m, max_step].
        """
        diff = desired - current
        if diff == 0:
            return current
        step = diff * gain
        if abs(step) >= max_step:
            step = math.copysign(max_step, step)
        elif abs(step) < 0.5:
            step = math.copysign(0.5, step)
        return current + step

    def wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle <= -math.pi:
            angle += 2 * math.pi
        while angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def adjust_angle(self, current_angle: float, desired_angle: float,
            max_step: float = 0.03, gain: float = 0.15) -> float:
        """
        Proportional yaw step via shortest path.
        Step = gain * angular error, clamped to [0.07 rad, max_step].
        """
        current_angle = self.wrap_angle(current_angle)
        desired_angle = self.wrap_angle(desired_angle)
        diff = self.wrap_angle(desired_angle - current_angle)
        if diff == 0:
            return current_angle
        step = diff * gain
        if abs(step) >= 0.07:
            step = math.copysign(max_step, step)
        else:
            step = math.copysign(0.07, step)
        new_angle = self.wrap_angle(current_angle + step)
        return new_angle

    def wrap_to_2pi(self, angle_rad: float) -> float:
        """Convert angle from [-pi, pi] to [0, 2pi]."""
        angle_rad = (angle_rad + math.pi) % (2 * math.pi) - math.pi
        return angle_rad % (2 * math.pi)

    def wrap_to_pi(self, angle_rad: float) -> float:
        """Convert angle from [0, 2pi] to [-pi, pi]."""
        angle_rad = angle_rad % (2 * math.pi)
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        return angle_rad

    # ------------------------------------------------------------------------
    # Mission management helpers
    # ------------------------------------------------------------------------

    def _load_next_waypoint(self) -> None:
        """
        Pop the next waypoint from the queue and reset navigation state for a
        fresh X -> Y traversal to the new target.

        After calling this, the timer_callback will:
          1. Rotate yaw toward new X.
          2. Fly to new X.
          3. Rotate yaw toward new Y.
          4. Fly to new Y.
        """
        self.forward_distance_x, self.forward_distance_y, self.takeoff_height = \
            self.traverse_coordinates_queue.get()

        # Set yaw to point toward new X immediately so rotation starts right away
        x_diff = self.forward_distance_x - self.vehicle_local_position.x
        self.actual_angle            = 0.0 if x_diff >= 0 else math.pi
        self.yaw_angle                = self.actual_angle
        self.actual_angle_difference = 0.0

        # Reset navigation flags: X must be done before Y
        self.x_achieved        = False
        self.x_rotate_achieved = False
        self.y_achieved        = True  # will be set False after X is done
        self.y_rotate_achieved = True

        self.forward_obstacle_target = ["x", 0.0, 0.0]
        self.intermittent_distance_x = self.vehicle_local_position.x
        self.intermittent_distance_y = self.vehicle_local_position.y

        self._log_event("WAYPOINT_LOADED",
            new_des_x=self.forward_distance_x,
            new_des_y=self.forward_distance_y,
            new_des_z=self.takeoff_height)

    def _advance_mission(self) -> None:
        """
        Called when the drone has fully completed the current navigation target
        (both X and Y achieved). Determines the next action based on mode:

          WAYPOINT mode, queue not empty   -> load and start next waypoint
          WAYPOINT mode, queue empty       -> land (mission complete)
          AUTONOMOUS mode, queue not empty -> follow user waypoint first, then resume
          AUTONOMOUS mode, queue empty     -> find next frontier
        """
        pos = self.vehicle_local_position

        # Signal "no active target"
        self.x_achieved              = True
        self.y_achieved              = True
        self.forward_obstacle_target = ["x", 0.0, 0.0]
        self.intermittent_distance_x = pos.x
        self.intermittent_distance_y = pos.y

        self._log_event("NAVIGATION_SEGMENT_COMPLETE")

        # Decision
        if not self.traverse_coordinates_queue.empty():
            # User-injected waypoints always take priority
            self._load_next_waypoint()
        elif self.drone_mode == DroneMode.AUTONOMOUS:
            # Find the next unexplored frontier
            self._set_next_autonomous_target()
        else:
            # WAYPOINT mode, nothing left: end mission
            self._log_event("MISSION_COMPLETE_LANDING")
            self.land()
            convert(self.log_file_name)
            exit(0)

    def _set_next_autonomous_target(self) -> None:
        """
        Find the nearest unexplored frontier and set it as the navigation target.
        If the exploration is complete (no frontiers left), land the drone.
        """
        frontier = self._find_nearest_frontier()

        if frontier is None:
            self._log_event("AUTONOMOUS_EXPLORATION_COMPLETE",
                total_grid_cells=len(self.occupancy_grid),
                visited_frontiers=len(self.auto_visited_cells))
            self.land()
            convert(self.log_file_name)
            exit(0)

        fx, fy = frontier

        # Set the frontier as the current navigation target
        self.forward_distance_x = fx
        self.forward_distance_y = fy
        # Keep existing altitude (takeoff_height) throughout exploration

        x_diff = fx - self.vehicle_local_position.x
        self.actual_angle            = 0.0 if x_diff >= 0 else math.pi
        self.yaw_angle                = self.actual_angle
        self.actual_angle_difference = 0.0

        self.x_achieved        = False
        self.x_rotate_achieved = False
        self.y_achieved        = True
        self.y_rotate_achieved = True

        self.forward_obstacle_target = ["x", 0.0, 0.0]
        self.intermittent_distance_x = self.vehicle_local_position.x
        self.intermittent_distance_y = self.vehicle_local_position.y

        self._log_event("AUTONOMOUS_NEW_FRONTIER_TARGET",
            frontier_x=round(fx, 3), frontier_y=round(fy, 3),
            frontiers_visited=len(self.auto_visited_cells),
            grid_cells_known=len(self.occupancy_grid))

    # ------------------------------------------------------------------------
    # Main control loop (100 Hz timer)
    # ------------------------------------------------------------------------

    def timer_callback(self) -> None:
        """
        100 Hz control loop. Phase structure:

          counter < 11   -> Increment counter (let PX4 see heartbeats)
          counter == 11  -> Engage offboard + arm; wait for confirmation
          counter == 12  -> Execute mission:
              z_achieved == False                       -> takeoff phase
              z_achieved == True, only_takeoff_and_land  -> land
              z_achieved == True, active target          -> run traversal state machine
              z_achieved == True, no active target       -> query mode for next action
        """
        self.publish_offboard_control_heartbeat_signal("position")
        # self._log_event("Control loop tick")

        # -- Pre-arm warm-up --
        if self.offboard_setpoint_counter == 11:
            # self.engage_offboard_mode()
            # self.arm()

            if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD :
                self.engage_offboard_mode()
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD \
            and self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
            if (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED):

                if not self.start_angle_initialized:
                    self.start_angle            = self.vehicle_local_position.heading
                    self.yaw_angle                = self.start_angle
                    self.actual_angle            = self.start_angle
                    self.start_angle_initialized = True
                    self._log_event("HEADING_ANGLE_INITALIZED",
                        des_yaw_rad=round(self.start_angle, 3)
                    )

                self.offboard_setpoint_counter += 1

                # Load the first waypoint immediately if one is waiting
                if not self.traverse_coordinates_queue.empty():
                    self._load_next_waypoint()
            else:
                self._log_event(f"WAITING_TO_BE_OFFBOARD_+_ARMED: NAV_STATE={self.vehicle_status.nav_state} ARMING_STATE={self.vehicle_status.arming_state}")

        # -- Mission execution --
        if not self.obstacle_found and self.offboard_setpoint_counter == 12:

            if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # PX4 not yet in offboard - keep waiting
                pass

            elif not self.z_achieved:
                # Phase: climb to takeoff altitude
                self._run_takeoff_phase()

            elif self.only_takeoff_and_land:
                # Special mode: takeoff then immediately land
                self._log_event("ONLY_TAKEOFF_AND_LAND_MODE_LANDING")
                self.land()
                convert(self.log_file_name)
                exit(0)

            elif not (self.x_achieved and self.y_achieved):
                # An active navigation target exists -> run traversal state machine
                self._run_traverse_state_machine()

            else:
                # No active target: decide what to do based on mode
                if not self.traverse_coordinates_queue.empty():
                    self._load_next_waypoint()
                elif self.drone_mode == DroneMode.AUTONOMOUS:
                    self._set_next_autonomous_target()
                else:
                    # WAYPOINT mode with empty queue - hover and wait
                    self._log_event("NO_WAYPOINT_RECEIVED_HOVERING")

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # ------------------------------------------------------------------------
        # Takeoff phase
        # ------------------------------------------------------------------------

    def _run_takeoff_phase(self) -> None:
        """
        Climb to takeoff_height while holding X=0, Y=0.
        Sets z_achieved=True once altitude is within +-0.25 m of target.
        """
        pos = self.vehicle_local_position

        if (round(pos.z, 2) <= round(self.takeoff_height, 2) - 0.25
                or round(pos.z, 2) >= round(self.takeoff_height, 2) + 0.25):
            # Still climbing
            interm_z    = self.adjust_position(pos.z, self.takeoff_height, max_step=0.9, gain=0.7)
            adjusted_x = self.adjust_position(pos.x, 0.00)
            adjusted_y = self.adjust_position(pos.y, 0.00)
            # Hold origin; use current heading to avoid unwanted yaw spin on takeoff
            self.publish_position_setpoint("position", 0.00, 0.00, interm_z, pos.heading)
            self._log_event("TAKEOFF_CLIMBING",
                target_z=self.takeoff_height,
                interm_z=round(interm_z, 3),
                adjusted_x=round(adjusted_x, 2),
                adjusted_y=round(adjusted_y, 2))

        elif (round(pos.z, 2) > round(self.takeoff_height, 2) - 0.25
                and round(pos.z, 2) < round(self.takeoff_height, 2) + 0.25):
            # Altitude achieved
            self.z_achieved = True
            # Pre-set yaw to point toward the first X target (minimises rotation time)
            if not (self.x_achieved and self.y_achieved):
                # A waypoint is already queued (_load_next_waypoint set it)
                pass
            else:
                # Autonomous mode: no target yet; point forward (yaw = 0)
                x_diff = self.forward_distance_x - pos.x
                self.actual_angle            = 0.0 if x_diff >= 0 else math.pi
                self.yaw_angle                = self.actual_angle
                self.actual_angle_difference = 0.0
            self._log_event("TAKEOFF_COMPLETE",
                initial_yaw_rad=round(self.yaw_angle, 3))

    # ------------------------------------------------------------------------
    # Traversal state machine  (shared by Mode 1 and Mode 2)
    # ------------------------------------------------------------------------

    def _run_traverse_state_machine(self) -> None:
        """
        4-step navigation toward the current (forward_distance_x, forward_distance_y):

        Step 1 Rotate yaw to face x direction.
        Step 2 Fly to X target (obstacle detour via forward_obstacle_target if needed).
        Step 3 Rotate yaw to face y direction.
        Step 4 Fly to Y target (obstacle detour via forward_obstacle_target if needed).

        On completion of Y target, calls advance_mission() which decides whether
        to load the next waypoint, find the next frontier, or land.

        Intermediate obstacle waypoints:
        When the LiDAR callback detects an obstacle mid-flight, it sets
        forward_obstacle_target[1/2] to a safe intermediate position.
        The state machine navigates there first, then continues to the original
        target (or marks the obstacle as active if still blocked).
        """
        pos = self.vehicle_local_position

        # --- Step 1: Rotate yaw toward x target
        if (not self.x_achieved and not self.x_rotate_achieved
            and abs(self.wrap_angle(pos.heading - self.yaw_angle)) > 0.02):

            self.intermittent_yaw = self.adjust_angle(
                pos.heading, self.yaw_angle, max_step=0.5, gain=0.15)
            self.publish_position_setpoint(
                "position",
                self.intermittent_distance_x, self.intermittent_distance_y,
                self.takeoff_height, self.intermittent_yaw)
            self._log_event("X_ROTATE_IN_PROGRESS",
                intern_yaw_rad=round(self.intermittent_yaw, 3),
                yaw_err_rad=round(self.wrap_angle(pos.heading - self.yaw_angle), 3))
        elif (not self.x_achieved and not self.x_rotate_achieved
              and abs(self.wrap_angle(pos.heading - self.yaw_angle)) <= 0.02):

            self.x_rotate_achieved = True
            self.intermittent_yaw = pos.heading
            self.forward_obstacle_target[1] = 0.0
            self.forward_obstacle_target[2] = 0.0
            self._log_event("X_ROTATE_COMPLETE",
                            final_yaw_rad=round(pos.heading, 3))

        # -- step 2a: Fly toward obstacle-dodge waypoint (X + Y) -------------------------
        elif (not self.x_achieved and self.x_rotate_achieved
              and self.forward_obstacle_target[1] != 0.0
              and self.forward_obstacle_target[2] != 0.0
              and (round(pos.x, 2) <= round(self.forward_obstacle_target[1], 2) - 0.25
                   or round(pos.x, 2) >= round(self.forward_obstacle_target[1], 2) + 0.25
                   or round(pos.y, 2) <= round(self.forward_obstacle_target[2], 2) - 0.25
                   or round(pos.y, 2) >= round(self.forward_obstacle_target[2], 2) + 0.25)):

            self.intermittent_distance_x = self.adjust_position(pos.x, self.forward_obstacle_target[1], max_step=0.9, gain=0.7)
            self.intermittent_distance_y = self.adjust_position(pos.y, self.forward_obstacle_target[2], max_step=0.9, gain=0.7)
            self.publish_position_setpoint(
                "position",
                self.intermittent_distance_x, self.intermittent_distance_y,
                self.takeoff_height, self.intermittent_yaw)
            self._log_event("X_MOVING_TO_OBSTACLE_DODGE_WP_XY")
        
        # -- step 2b: Arrived atobstacle-dodge waypoint (X + Y) -------------------------
        elif (not self.x_achieved and self.x_rotate_achieved
                and self.forward_obstacle_target[1] != 0.0
                and self.forward_obstacle_target[2] != 0.0
                and round(pos.x, 2) > round(self.forward_obstacle_target[1], 2) - 0.25
                and round(pos.x, 2) < round(self.forward_obstacle_target[1], 2) + 0.25
                and round(pos.y, 2) > round(self.forward_obstacle_target[2], 2) - 0.25
                and round(pos.y, 2) < round(self.forward_obstacle_target[2], 2) + 0.25):

            self._log_event("X_REACHED_OBSTACLE_DODGE_WP_XY")
            if not self.check_destination("x"):
                self.forward_obstacle_target[0] = "x"
                self.obstacle_found = True

        # -- step 2c: Fly toward obstacle-dodge waypoint (X only) -------------------------
        elif (not self.x_achieved and self.x_rotate_achieved
                and self.forward_obstacle_target[1] != 0.0
                and self.forward_obstacle_target[2] == 0.0
                and (round(pos.x, 2) <= round(self.forward_obstacle_target[1], 2) - 0.25
                    or round(pos.x, 2) >= round(self.forward_obstacle_target[1], 2) + 0.25)):

            self.intermittent_distance_x = self.adjust_position(pos.x, self.forward_obstacle_target[1], max_step=0.9, gain=0.7)
            self.publish_position_setpoint(
                "position",
                self.intermittent_distance_x, self.intermittent_distance_y,
                self.takeoff_height, self.intermittent_yaw)
            self._log_event("X_MOVING_TO_OBSTACLE_DODGE_WP_X_ONLY")

        # -- step 2d: Arrived at obstacle-dodge waypoint (X only) -------------------------
        elif (not self.x_achieved and self.x_rotate_achieved
                and self.forward_obstacle_target[1] != 0.0
                and self.forward_obstacle_target[2] == 0.0
                and round(pos.x, 2) > round(self.forward_obstacle_target[1], 2) - 0.25
                and round(pos.x, 2) < round(self.forward_obstacle_target[1], 2) + 0.25):

            self._log_event("X_REACHED_OBSTACLE_DODGE_WP_X_ONLY")
            if not self.check_destination("x"):
                self.forward_obstacle_target[0] = "x"
                self.obstacle_found = True

        # -- step 2e: Fly directly to X target -------------------------
        elif (not self.x_achieved and self.x_rotate_achieved
                and self.forward_obstacle_target[1] == 0.0
                and self.forward_obstacle_target[2] == 0.0
                and (round(pos.x, 2) <= round(self.forward_distance_x, 2) - 0.25
                    or round(pos.x, 2) >= round(self.forward_distance_x, 2) + 0.25)):

            self.intermittent_distance_x = self.adjust_position(pos.x, self.forward_obstacle_target[1], max_step=0.9, gain=0.7)
            self.publish_position_setpoint(
                "position",
                self.intermittent_distance_x, self.intermittent_distance_y,
                self.takeoff_height, self.intermittent_yaw)
            self._log_event("X_MOVING_TO_TARGET",
                           x_diff_remaining=round(self.forward_distance_x - pos.x, 3))

        # -- step 2f: X target reached -------------------------
        elif (not self.x_achieved and self.x_rotate_achieved
                and self.forward_obstacle_target[1] == 0.0
                and self.forward_obstacle_target[2] == 0.0
                and round(pos.x, 2) > round(self.forward_distance_x, 2) - 0.25
                and round(pos.x, 2) < round(self.forward_distance_x, 2) + 0.25):

            self.x_achieved = True
            # Set yaw towards Y target
            self.actual_angle = (
                math.pi / 2 if (self.self.forward_distance_y - pos.y) >= 0
                else -math.pi / 2
            )
            self.yaw_angle = self.actual_angle
            self.actual_angle_difference = 0.0
            self.forward_obstacle_target[1] = 0.0
            self.forward_obstacle_target[2] = 0.0
            self.intermittent_distance_x = pos.x
            self.intermittent_distance_y = pos.y

            self._log_event("X_TARGET_REACHED",
            new_yaw_rad = round(self.yaw_angle, 3),
            y_diff_remaining=round(self.forward_distance_y - pos.y, 3))

            if (pos.y < self.forward_distance_y - 0.25
                or pos.y > self.forward_distance_y + 0.25):
                #Y still need to be traversed
                self.y_achieved = False
                self.y_rotate_achieved = False
                self.publish_position_setpoint("position",
                                               self.intermittent_distance_x,self.intermittent_distance_y,
                                               self.takeoff_height, self.yaw_angle)
            else:
                # Y already within tolerance -> mission segment complete
                self._advance_mission()

        
        # Step 3: Rotate yaw toward Y target
        elif (not self.y_achieved and not self.y_rotate_achieved
              and abs(self.wrap_angle(pos.heading - self.yaw_angle)) > 0.02):

            adj_yaw = self.adjust_angle(pos.heading, self.yaw_angle, max_step=0.5, gain=0.15)
            self.publish_position_setpoint("position",
                self.intermittent_distance_x, self.intermittent_distance_y,
                self.takeoff_height, adj_yaw)
            self._log_event("Y_ROTATE_IN_PROGRESS",
                interm_yaw_rad=round(adj_yaw, 3),
                yaw_err_rad=round(abs(self.wrap_angle(pos.heading - self.yaw_angle)), 3))

        elif (not self.y_achieved and not self.y_rotate_achieved
              and abs(self.wrap_angle(pos.heading - self.yaw_angle)) <= 0.02):

            self.y_rotate_achieved = True
            self.forward_obstacle_target[1] = 0.0
            self.forward_obstacle_target[2] = 0.0
            self._log_event("Y_ROTATE_COMPLETED",
                final_yaw_rad=round(pos.heading, 3))

        # — Step 4a: Fly toward obstacle-dodge waypoint (X + Y) —
        elif (not self.y_achieved and self.y_rotate_achieved
              and self.forward_obstacle_target[1] != 0.0
              and self.forward_obstacle_target[2] != 0.0
              and (round(pos.x, 2) == round(self.forward_obstacle_target[1], 2) - 0.25
                   or round(pos.x, 2) == round(self.forward_obstacle_target[1], 2) + 0.25)
              and (round(pos.y, 2) == round(self.forward_obstacle_target[2], 2) - 0.25
                   or round(pos.y, 2) == round(self.forward_obstacle_target[2], 2) + 0.25)):

            self.intermittent_distance_x = self.adjust_position(pos.x, self.forward_obstacle_target[1], max_step=0.9, gain=0.7)
            self.intermittent_distance_y = self.adjust_position(pos.y, self.forward_obstacle_target[2], max_step=0.9, gain=0.7)
            self.publish_position_setpoint(
                self.intermittent_distance_x,
                self.takeoff_height, self.intermittent_distance_y)
            self._log_event("Y_MOVING_TO_OBSTACLE_DODGE_WP_XY")

        # — Step 4b: Arrived at obstacle-dodge waypoint (X + Y) —
        elif (not self.y_achieved and self.y_rotate_achieved
              and self.forward_obstacle_target[1] != 0.0
              and self.forward_obstacle_target[2] != 0.0
              and round(pos.x, 2) == round(self.forward_obstacle_target[1], 2)
              and round(pos.y, 2) == round(self.forward_obstacle_target[2], 2)):

            self._log_event("Y_REACHED_OBSTACLE_DODGE_WP_XY")
            if not self.check_destination("y"):
                self.forward_obstacle_target[0] = "y"
                self.obstacle_found = True

        # — Step 4c: Fly toward obstacle-dodge waypoint (Y only) —
        elif (not self.y_achieved and self.y_rotate_achieved
              and self.forward_obstacle_target[1] == 0.0
              and self.forward_obstacle_target[2] != 0.0
              and (round(pos.y, 2) < round(self.forward_obstacle_target[2], 2) - 0.25
                   or round(pos.y, 2) > round(self.forward_obstacle_target[2], 2) + 0.25)):

            self.intermittent_distance_y = self.adjust_position(pos.y, self.forward_obstacle_target[2], max_step=0.9, gain=0.7)
            self.publish_position_setpoint("position",
                                           self.intermittent_distance_x, self.intermittent_distance_y,
                                           self.takeoff_height, self.yaw_angle)
            self._log_event("Y_MOVING_TO_OBSTACLE_DODGE_WP_Y_ONLY",
                            y_diff_remaining=round(self.forward_obstacle_target[2] - pos.y, 3))

        # --- Step 4d: Arrived at obstacle-dodge waypoint (Y only) ---
        elif (not self.y_achieved and self.y_rotate_achieved
              and self.forward_obstacle_target[1] == 0.0
              and self.forward_obstacle_target[2] != 0.0
              and round(pos.y, 2) < round(self.forward_obstacle_target[2], 2) - 0.25
              and round(pos.y, 2) < round(self.forward_obstacle_target[2], 2) + 0.25):

            self._log_event("Y_REACHED_OBSTACLE_DODGE_WP_Y_ONLY")
            if not self.check_destination("y"):
                self.forward_obstacle_target[0] = "y"
                self.obstacle_found = True

        # --- Step 4e: Fly directly to Y target ---
        elif (not self.y_achieved and self.y_rotate_achieved
              and self.forward_obstacle_target[1] == 0.0
              and self.forward_obstacle_target[2] == 0.0
              and (round(pos.y, 2) < round(self.forward_distance_y, 2) - 0.25
                   or round(pos.y, 2) > round(self.forward_distance_y, 2) + 0.25)):

            self.intermittent_distance_y = self.adjust_position(pos.y, self.forward_distance_y, max_step=0.9, gain=0.7)
            self.publish_position_setpoint("position",
                                           self.intermittent_distance_x, self.intermittent_distance_y,
                                           self.takeoff_height, self.yaw_angle)
            self._log_event("Y_MOVING_TO_TARGET",
                            y_diff_remaining=round(self.forward_distance_y - pos.y, 3))

        # --- Step 4f: Y target reached ---
        elif (not self.y_achieved and self.y_rotate_achieved
              and self.forward_obstacle_target[1] == 0.0
              and self.forward_obstacle_target[2] == 0.0
              and round(pos.y, 2) > -0.25
              and round(pos.y, 2) < round(self.forward_distance_y, 2) + 0.25):

            self.y_achieved = True
            # Pre-set yaw for the next traversal (used when loading next target)
            self.actual_angle = (
                0.0 if (self.forward_distance_x - pos.x) >= 0 else math.pi
            )

            self.yaw_angle = self.actual_angle
            self.actual_angle_difference = 0.0
            self.forward_obstacle_target[1] = 0.0
            self.forward_obstacle_target[2] = 0.0
            self.intermittent_distance_x = pos.x
            self.intermittent_distance_y = pos.y

            self._log_event("Y_TARGET_REACHED",
                            new_yaw_rad=round(self.yaw_angle, 3),
                            x_diff_remaining=round(self.forward_distance_x - pos.x, 3))

            if (pos.x < self.forward_distance_x * 0.25
                or pos.x > self.forward_distance_x * 0.25):
                # X was alread done for this waypoint but now needs traversal again
                # (happens when next waypoint has a different x)
                self.x_achieved = False
                self.x_rotate_achieved = False
                self.publish_position_setpoint("position",
                                               self.intermittent_distance_x, self.intermittent_distance_y,
                                               self.takeoff_height, self.yaw_angle)
            else:
                # Both X and Y complete for the target
                self._advance_mission()
    
    # ---------------------------------------------
    # Destination check helper
    # ---------------------------------------------

    def check_destination(self, direction: str) -> bool:
        """
        Called when the drone has reached an obstacle-avoidance intermediate
        waypoint. Checks whether the ORIGINAL destination is also already
        within tolerance.

        If yes: mark the axis done, set up the next axis or call _advance_mission().
            Returns True (avoidance loop can stop).
        If no: the original destination is not yet reached (obstacle blocking).
            Returns False (caller should activate obstacle_found).
        """
        pos = self.vehicle_local_position

        if direction == "x":
            if (not self.x_achieved and self.x_rotate_achieved
                and round(pos.x, 2) > round(self.forward_distance_x, 2) - 0.25
                and round(pos.x, 2) < round(self.forward_distance_x, 2) + 0.25):

                self.x_achieved = True
                self.actual_angle = (
                    math.pi / 2 if (self.forward_distance_y - pos.y) >= 0
                    else -math.pi / 2
                )
                self.yaw_angle = self.actual_angle
                self.actual_angle_difference = 0.0
                self.forward_obstacle_target[1] = 0.0
                self.forward_obstacle_target[2] = 0.0
                self.intermittent_distance_x = pos.x
                self.intermittent_distance_y = pos.y

                self._log_event("CHECK_DEST_X_ALREADY_REACHED",
                                new_yaw_rad=round(self.yaw_angle, 3),
                                y_diff_remaining=round(self.forward_distance_y - pos.y, 3))

                if (pos.y < self.forward_distance_y - 0.25
                    or pos.y > self.forward_distance_y + 0.25):
                    self.y_achieved = False
                    self.y_rotate_achieved = False
                    self.publish_position_setpoint("position",
                    self.intermittent_distance_x, self.intermittent_distance_y,
                    self.takeoff_height, self.yaw_angle)
                else:
                    self._advance_mission()
                return True

        elif direction == "y":
            if (not self.y_achieved and self.y_rotate_achieved
                and round(pos.y, 2) > round(self.forward_distance_y, 2) - 0.25
                and round(pos.y, 2) < round(self.forward_distance_yy, 2) + 0.25):

                self.y_achieved = True
                self.actual_angle = (
                    0.0 if (self.forward_distance_x - pos.x) >= 0
                    else math.pi
                )
                self.yaw_angle = self.actual_angle
                self.actual_angle_difference = 0.0
                self.forward_obstacle_target[1] = 0.0
                self.forward_obstacle_target[2] = 0.0
                self.intermittent_distance_x = pos.x
                self.intermittent_distance_y = pos.y

                self._log_event("CHECK_DEST_Y_ALREADY_REACHED",
                                new_yaw_rad=round(self.yaw_angle, 3),
                                x_diff_remaining=round(self.forward_distance_x - pos.x, 3))

                if (pos.x < self.forward_distance_x - 0.25
                    or pos.x > self.forward_distance_x + 0.25):
                    self.x_achieved = False
                    self.x_rotate_achieved = False
                    self.publish_position_setpoint("position",
                    self.intermittent_distance_x, self.intermittent_distance_y,
                    self.takeoff_height, self.yaw_angle)
                else:
                    self._advance_mission()
                return True
        return False

def main(args=None) -> None:
    print('Started offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()