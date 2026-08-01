// ==========================================================================
// odom_transform.cpp
//
// ROS2 Node: OdomTransform
//
// Purpose:
//   This node bridges PX4 autopilot data with ROS2 navigation stack (SLAM/Nav2).
//   It handles three data flows:
//
//   1. PX4 -> ROS (odometryCallback, ~50-250 Hz)
//      PX4 VehicleOdometry (NED/FRD) -> ENU/FLU odometry for Nav2 + TF broadcast
//
//   2. LiDAR -> ROS (callback_scan, rate depends on sensor)
//      Re-stamps LaserScan and broadcasts the base_footprint -> laser TF
//
//   3. SLAM -> PX4 (posecallback + evaluate_and_publish_odometry)
//      SLAM PoseWithCovarianceStamped (ENU) -> PX4 VehicleOdometry (NED/FRD)
//      for Visual Odometry (EKF2) fusion. A 200 Hz timer re-sends the last
//      SLAM pose to satisfy PX4's minimum VIO rate requirement.
//
// Frame conventions:
//   PX4  : NED position (X=North, Y=East, Z=Down), FRD body (roll-right, pitch-down)
//   ROS2 : ENU position (X=East,  Y=North, Z=Up),  FLU body (roll-right, pitch-up)
//
// Logging tags used (searchable prefix for filtering or plotting):
//   [INIT]       - node startup info
//   [PX4->ENU]   - drone state from PX4 odometry (good for plotting drone pose)
//   [SCAN]       - LiDAR scan received / re-published
//   [SLAM->PX4]  - SLAM pose converted and sent to PX4 as VIO
//   [VO INIT]    - first VO message sent before any SLAM data arrives
//   [VO REPUB]   - periodic re-publish of cached SLAM pose (DEBUG level)
//   [OFFSET]     - frame-alignment offset (reserved for future use)
// ==========================================================================

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <vector>

using namespace px4_msgs::msg;

// ==========================================================================

class OdomTransform : public rclcpp::Node {
public:
    OdomTransform()
        : Node("odom_transform"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          offset_initialized_(false)
    {
        // ------------------------------------------------------------------------
        // Declare all ROS2 parameters with sensible defaults.
        // Override these in a launch file or YAML config.
        // ------------------------------------------------------------------------
        declare_parameter<std::string>("base_frame_id",               "base_footprint");
        declare_parameter<std::string>("scan_child_frame_id",         "laser");
        declare_parameter<std::string>("odom_frame_id",               "odom");
        declare_parameter<std::string>("scan_subscribe_topic",        "/scan");
        declare_parameter<std::string>("scan_publish_topic",          "/scan_modified");
        declare_parameter<std::string>("odom_subscribe_topic",        "/fmu/out/vehicle_odometry");
        declare_parameter<std::string>("odom_publish_topic",          "/odom");
        declare_parameter<std::string>("pose_subscribe_topic",        "/pose");
        declare_parameter<std::string>("external_odom_publish_topic", "/fmu/in/vehicle_visual_odometry");
        declare_parameter<double>("last_addition", 0.02);

        base_frame_id_               = get_parameter("base_frame_id").as_string();
        scan_child_frame_id_         = get_parameter("scan_child_frame_id").as_string();
        odom_frame_id_               = get_parameter("odom_frame_id").as_string();
        scan_subscribe_topic_        = get_parameter("scan_subscribe_topic").as_string();
        scan_publish_topic_          = get_parameter("scan_publish_topic").as_string();
        odom_subscribe_topic_        = get_parameter("odom_subscribe_topic").as_string();
        odom_publish_topic_          = get_parameter("odom_publish_topic").as_string();
        pose_subscribe_topic_        = get_parameter("pose_subscribe_topic").as_string();
        external_odom_publish_topic_ = get_parameter("external_odom_publish_topic").as_string();
        last_addition_               = get_parameter("last_addition").as_double();

        // BestEffort QoS matches PX4 uXRCE-DDS publisher profile
        rclcpp::QoS qos_best_effort(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // ------------------------------------------------------------------------
        // Subscribers
        // ------------------------------------------------------------------------

        // PX4 fused vehicle odometry in NED/FRD - drives the ROS TF tree
        odom_subscription_ = this->create_subscription<VehicleOdometry>(
            odom_subscribe_topic_, qos_best_effort,
            std::bind(&OdomTransform::odometryCallback, this, std::placeholders::_1));

        // Raw 2-D LiDAR scan - re-stamped and re-published under the laser frame
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_subscribe_topic_, qos_best_effort,
            std::bind(&OdomTransform::callback_scan, this, std::placeholders::_1));

        // SLAM pose estimate in ENU - converted to NED/FRD and fed back to PX4
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_subscribe_topic_, 10,
            std::bind(&OdomTransform::posecallback, this, std::placeholders::_1));

        // ------------------------------------------------------------------------
        // Publishers
        // ------------------------------------------------------------------------
        scan_publisher_          = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_publish_topic_, 10);
        odom_publisher_          = this->create_publisher<nav_msgs::msg::Odometry>(odom_publish_topic_, 10);
        external_odom_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(external_odom_publish_topic_, 10);

        // TF broadcaster (used by both odom and scan callbacks)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // ------------------------------------------------------------------------
        // 200 Hz timer - keeps PX4 fed with VIO even when SLAM is slower.
        // PX4 EKF2 expects VIO at >= ~50 Hz; this timer satisfies that by
        // re-sending the last cached SLAM pose with a fresh timestamp.
        // ------------------------------------------------------------------------
        publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&OdomTransform::evaluate_and_publish_odometry, this));

        RCLCPP_INFO(this->get_logger(),
            "[INIT] OdomTransform ready | "
            "odom_in='%s'  odom_out='%s' | "
            "scan_in='%s'  scan_out='%s' | "
            "slam_in='%s'  vio_out='%s'",
            odom_subscribe_topic_.c_str(),       odom_publish_topic_.c_str(),
            scan_subscribe_topic_.c_str(),       scan_publish_topic_.c_str(),
            pose_subscribe_topic_.c_str(),       external_odom_publish_topic_.c_str());
    }

private:

    // ==========================================================================
    // Utility helpers
    // ==========================================================================

    // Returns current wall-clock time as "YYYY-MM-DD HH:MM:SS.mmm"
    std::string getTimestamp() const
    {
        auto now = std::chrono::system_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&now_c);
        std::ostringstream ss;
        ss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    // Extracts yaw (radians) from a quaternion using the ENU/ROS convention
    double yawFromQuatRad(const Eigen::Quaterniond& q) const
    {
        const double num = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double den = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        return std::atan2(num, den);
    }

    double radToDeg(double rad) const { return rad * 180.0 / M_PI; }

    // Returns a human-readable string of cardinal-direction LiDAR distances.
    // Assumes the scan covers 360deg with index 0 at angle_min.
    // Returns "lidar=N/A" if no scan has been received yet.
    std::string lidarSummary() const
    {
        if (!has_latest_scan_ || latest_scan_ranges_.empty()) {
            return "lidar=N/A";
        }

        // Format a single range value, printing "inf" for non-finite readings
        auto fmt = [](float v) -> std::string {
            if (std::isfinite(v)) {
                std::ostringstream s;
                s << std::fixed << std::setprecision(2) << v << "m";
                return s.str();
            }
            return "inf";
        };

        const size_t n       = latest_scan_ranges_.size();
        const size_t i_front = 0;          // 0deg   - forward
        const size_t i_right = n / 4;      // 90deg  - right
        const size_t i_back  = n / 2;      // 180deg - rearward
        const size_t i_left  = 3 * n / 4;  // 270deg - left

        std::ostringstream ss;
        ss << "lidar(cardinal): "
           << "front=" << fmt(latest_scan_ranges_[i_front])
           << " right=" << fmt(latest_scan_ranges_[i_right])
           << " back="  << fmt(latest_scan_ranges_[i_back])
           << " left="  << fmt(latest_scan_ranges_[i_left]);
        return ss.str();
    }

    // ==========================================================================
    // Callback 1: PX4 VehicleOdometry (NED/FRD)
    //
    // What it does:
    //   Receives the autopilot's fused pose estimate, converts it to the ROS
    //   ENU/FLU convention, and publishes nav_msgs/Odometry + odom->base TF.
    //
    // Key conversions:
    //   Position:    NED (X=N, Y=E, Z=D) -> ENU (X=E, Y=N, Z=U)
    //                x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
    //   Orientation: NED/FRD quaternion -> ENU/FLU quaternion
    //                via px4_ros_com frame_transforms
    //   Velocity:    same NED->ENU swap
    // ==========================================================================
    void odometryCallback(const VehicleOdometry::SharedPtr msg)
    {
        // --- Position: NED -> ENU ---
        const Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
        const Eigen::Vector3d pos_enu(pos_ned.y(), pos_ned.x(), -pos_ned.z());

        // --- Orientation: NED/FRD -> ENU/FLU ---
        const Eigen::Quaterniond q_raw =
            px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
        const Eigen::Quaterniond q_enu_frame =
            px4_ros_com::frame_transforms::ned_to_enu_orientation(q_raw);
        Eigen::Quaterniond q_ros =
            px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(q_enu_frame);
        q_ros.normalize();

        // --- Velocity: NED -> ENU ---
        const Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        const Eigen::Vector3d vel_enu(vel_ned.y(), vel_ned.x(), -vel_ned.z());

        // Yaw in both NED (raw) and ENU (converted) for diagnostics and plotting
        const double yaw_ned_deg = radToDeg(yawFromQuatRad(q_raw));
        const double yaw_enu_deg = radToDeg(yawFromQuatRad(q_ros));

        // Cache drone state so posecallback can log the SLAM<->drone position difference
        latest_drone_pos_enu_ = pos_enu;
        latest_drone_yaw_deg_ = yaw_enu_deg;
        has_drone_pos_        = true;

        // Cache the latest PX4 quaternion (NED/FRD) for the 200 Hz VIO re-publish timer
        latest_px4_q_ = msg->q;
        has_px4_q_    = true;

        // --- Publish nav_msgs/Odometry ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp             = this->get_clock()->now();
        odom_msg.header.frame_id          = odom_frame_id_;
        odom_msg.child_frame_id           = base_frame_id_;
        odom_msg.pose.pose.position.x     = pos_enu.x();
        odom_msg.pose.pose.position.y     = pos_enu.y();
        odom_msg.pose.pose.position.z     = pos_enu.z();
        odom_msg.pose.pose.orientation.x  = q_ros.x();
        odom_msg.pose.pose.orientation.y  = q_ros.y();
        odom_msg.pose.pose.orientation.z  = q_ros.z();
        odom_msg.pose.pose.orientation.w  = q_ros.w();
        odom_msg.twist.twist.linear.x     = vel_enu.x();
        odom_msg.twist.twist.linear.y     = vel_enu.y();
        odom_msg.twist.twist.linear.z     = vel_enu.z();
        odom_publisher_->publish(odom_msg);

        // --- Broadcast TF: odom -> base_footprint ---
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp            = odom_msg.header.stamp;
        odom_tf.header.frame_id         = odom_frame_id_;
        odom_tf.child_frame_id          = base_frame_id_;
        odom_tf.transform.translation.x = pos_enu.x();
        odom_tf.transform.translation.y = pos_enu.y();
        odom_tf.transform.translation.z = pos_enu.z();
        odom_tf.transform.rotation      = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(odom_tf);

        // --- Compute SLAM<->drone position difference (when SLAM has been received) ---
        std::string diff_str;
        if (has_slam_enu_pos_) {
            const double dx    = latest_slam_enu_pos_.x() - pos_enu.x();
            const double dy    = latest_slam_enu_pos_.y() - pos_enu.y();
            const double dz    = latest_slam_enu_pos_.z() - pos_enu.z();
            const double d_yaw = latest_slam_yaw_deg_      - yaw_enu_deg;
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "diff(slam-drone): dx=" << dx
               << " dy=" << dy
               << " dz=" << dz
               << " dyaw=" << d_yaw << "deg";
            diff_str = ss.str();
        } else {
            diff_str = "diff=N/A(awaiting_SLAM)";
        }

        // --- Structured log: drone state, raw NED, velocity, quaternions, diff, LiDAR ---
        RCLCPP_INFO(this->get_logger(),
            "[PX4->ENU] ts=%s | "
            "drone(ENU): x=%.3f y=%.3f z=%.3f yaw=%.2fdeg | "
            "raw(NED): x=%.3f y=%.3f z=%.3f yaw=%.2fdeg | "
            "vel(ENU): vx=%.3f vy=%.3f vz=%.3f | "
            "quat_ned(x=%.4f y=%.4f z=%.4f w=%.4f) "
            "quat_enu(x=%.4f y=%.4f z=%.4f w=%.4f) | "
            "%s | %s",
            getTimestamp().c_str(),
            pos_enu.x(), pos_enu.y(), pos_enu.z(), yaw_enu_deg,
            pos_ned.x(), pos_ned.y(), pos_ned.z(), yaw_ned_deg,
            vel_enu.x(), vel_enu.y(), vel_enu.z(),
            msg->q[0], msg->q[1], msg->q[2], msg->q[3],
            q_ros.x(), q_ros.y(), q_ros.z(), q_ros.w(),
            diff_str.c_str(),
            lidarSummary().c_str());
    }

    // ==========================================================================
    // Callback 2: LaserScan (2-D LiDAR)
    //
    // What it does:
    //   Re-stamps the incoming scan to current node time (prevents stale-TF
    //   warnings in Nav2/SLAM), updates the frame_id, and republishes.
    //   Also broadcasts the static base_footprint -> laser TF so the scan can
    //   be overlaid on the robot body in RViz and used by SLAM.
    // ==========================================================================
    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Cache the full scan for LiDAR summary in other logs
        latest_scan_ranges_    = msg->ranges;
        latest_scan_angle_min_ = msg->angle_min;
        latest_scan_angle_max_ = msg->angle_max;
        latest_scan_range_min_ = msg->range_min;
        latest_scan_range_max_ = msg->range_max;
        has_latest_scan_       = true;

        // Re-stamp and re-publish under the laser frame
        auto modified_scan = *msg;
        modified_scan.header.stamp    = this->get_clock()->now();
        modified_scan.header.frame_id = scan_child_frame_id_;
        scan_publisher_->publish(modified_scan);

        // Broadcast TF: base_footprint -> laser
        // Update translation/rotation offsets to match physical LiDAR mounting position
        geometry_msgs::msg::TransformStamped laser_tf;
        laser_tf.header.stamp            = modified_scan.header.stamp;
        laser_tf.header.frame_id         = base_frame_id_;
        laser_tf.child_frame_id          = scan_child_frame_id_;
        laser_tf.transform.translation.x = 0.0;  // [m] forward offset from CoM - adjust if needed
        laser_tf.transform.translation.y = 0.0;  // [m] lateral offset
        laser_tf.transform.translation.z = 0.0;  // [m] vertical offset
        laser_tf.transform.rotation.x    = 0.0;
        laser_tf.transform.rotation.y    = 0.0;
        laser_tf.transform.rotation.z    = 0.0;
        laser_tf.transform.rotation.w    = 1.0;  // identity - LiDAR is level and forward-facing
        tf_broadcaster_->sendTransform(laser_tf);

        RCLCPP_INFO(this->get_logger(),
            "[SCAN] ts=%s | "
            "Re-published scan -> topic='%s' frame='%s' | "
            "rays=%zu angle=[%.1f,%.1f]deg range=[%.2f,%.2f]m | "
            "%s",
            getTimestamp().c_str(),
            scan_publish_topic_.c_str(), scan_child_frame_id_.c_str(),
            msg->ranges.size(),
            radToDeg(msg->angle_min), radToDeg(msg->angle_max),
            msg->range_min, msg->range_max,
            lidarSummary().c_str());
    }

    // ==========================================================================
    // Callback 3: SLAM PoseWithCovarianceStamped (ENU frame)
    //
    // What it does:
    //   Receives the SLAM localization estimate (e.g. from slam_toolbox or
    //   RTAB-Map) and converts it to the PX4 NED/FRD convention so PX4's
    //   EKF2 can fuse it as Visual Odometry (VIO).
    //
    // Key conversions:
    //   Position:    ENU -> NED  (x_ned = y_enu, y_ned = x_enu, z_ned = 0)
    //                NOTE: z is intentionally zeroed - PX4 fuses its own
    //                altitude from baro/rangefinder, not from 2-D SLAM.
    //   Orientation: ENU/FLU -> NED/FRD  via px4_ros_com frame_transforms
    // ==========================================================================
    void posecallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        const rclcpp::Time now = this->get_clock()->now();

        // --- Position: ENU -> NED (Z zeroed - PX4 handles altitude independently) ---
        const Eigen::Vector3d pos_enu(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
        const Eigen::Vector3d pos_ned(pos_enu.y(), pos_enu.x(), 0.0);

        // --- Orientation: ENU/FLU -> NED/FRD ---
        const Eigen::Quaterniond q_enu(msg->pose.pose.orientation.w,
                                        msg->pose.pose.orientation.x,
                                        msg->pose.pose.orientation.y,
                                        msg->pose.pose.orientation.z);
        const Eigen::Quaterniond q_ned_frame =
            px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        Eigen::Quaterniond q_px4 =
            px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(q_ned_frame);
        q_px4.normalize();

        // Yaw in both ENU (SLAM input) and NED (PX4 output) for diagnostics
        const double yaw_enu_deg = radToDeg(yawFromQuatRad(q_enu));
        const double yaw_ned_deg = radToDeg(yawFromQuatRad(q_px4));

        // Cache SLAM ENU state so odometryCallback can compute the drone<->SLAM difference
        latest_slam_enu_pos_ = pos_enu;
        latest_slam_yaw_deg_ = yaw_enu_deg;
        has_slam_enu_pos_    = true;

        // --- Build px4_msgs/VehicleOdometry ---
        px4_msgs::msg::VehicleOdometry vo;
        vo.timestamp        = now.nanoseconds() / 1000;  // microseconds
        vo.timestamp_sample = vo.timestamp;
        vo.pose_frame       = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        vo.velocity_frame   = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;
        vo.position = {
            static_cast<float>(pos_ned.x()),
            static_cast<float>(pos_ned.y()),
            static_cast<float>(pos_ned.z())
        };
        // PX4 quaternion array order: [x, y, z, w]
        vo.q = {
            static_cast<float>(q_px4.x()),
            static_cast<float>(q_px4.y()),
            static_cast<float>(q_px4.z()),
            static_cast<float>(q_px4.w())
        };
        // Position variance from SLAM covariance diagonal.
        // Z variance is large (25.0) because z is not trusted from 2-D SLAM.
        vo.position_variance = {
            static_cast<float>(msg->pose.covariance[0]),  // var(x)
            static_cast<float>(msg->pose.covariance[7]),  // var(y)
            25.0f                                          // var(z) - intentionally large
        };
        // Orientation variance (roll, pitch, yaw diagonal of 6x6 covariance)
        vo.orientation_variance = {
            static_cast<float>(msg->pose.covariance[21]),  // var(roll)
            static_cast<float>(msg->pose.covariance[28]),  // var(pitch)
            static_cast<float>(msg->pose.covariance[35])   // var(yaw)
        };
        vo.quality = 100;

        // Cache for the 200 Hz re-publish timer
        last_slam_pose_      = vo;
        last_slam_pose_time_ = now;
        has_slam_pose_       = true;

        // Publish immediately (direct delivery on every new SLAM update)
        external_odom_publisher_->publish(vo);

        // --- Compute SLAM<->drone position difference (when drone pose available) ---
        std::string diff_str;
        if (has_drone_pos_) {
            const double dx    = pos_enu.x() - latest_drone_pos_enu_.x();
            const double dy    = pos_enu.y() - latest_drone_pos_enu_.y();
            const double dz    = pos_enu.z() - latest_drone_pos_enu_.z();
            const double d_yaw = yaw_enu_deg  - latest_drone_yaw_deg_;
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "diff(slam-drone): dx=" << dx
               << " dy=" << dy
               << " dz=" << dz
               << " dyaw=" << d_yaw << "deg";
            diff_str = ss.str();
        } else {
            diff_str = "diff=N/A(awaiting_PX4_odom)";
        }

        // --- Structured log: SLAM input, PX4 output, quaternions, variance, diff, LiDAR ---
        RCLCPP_INFO(this->get_logger(),
            "[SLAM->PX4] ts=%s | "
            "slam_in(ENU): x=%.3f y=%.3f z=%.3f yaw=%.2fdeg | "
            "vio_out(NED): x=%.3f y=%.3f z=%.3f yaw=%.2fdeg | "
            "quat_enu(x=%.4f y=%.4f z=%.4f w=%.4f) "
            "quat_ned(x=%.4f y=%.4f z=%.4f w=%.4f) | "
            "pos_var=[%.4f %.4f %.4f] ori_var=[%.4f %.4f %.4f] quality=%d | "
            "%s | %s",
            getTimestamp().c_str(),
            pos_enu.x(), pos_enu.y(), pos_enu.z(), yaw_enu_deg,
            pos_ned.x(), pos_ned.y(), pos_ned.z(), yaw_ned_deg,
            q_enu.x(), q_enu.y(), q_enu.z(), q_enu.w(),
            q_px4.x(), q_px4.y(), q_px4.z(), q_px4.w(),
            vo.position_variance[0], vo.position_variance[1], vo.position_variance[2],
            vo.orientation_variance[0], vo.orientation_variance[1], vo.orientation_variance[2],
            vo.quality,
            diff_str.c_str(),
            lidarSummary().c_str());
    }

    // ==========================================================================
    // Timer: ~200 Hz VIO re-publisher
    //
    // What it does:
    //   PX4 EKF2 requires VIO messages at a minimum of ~50 Hz. SLAM toolkits
    //   often run at only 5-30 Hz. This timer fills the gap by re-sending the
    //   last cached SLAM pose with an updated timestamp.
    //
    //   Orientation override: when the drone is moving but SLAM has stalled,
    //   substituting the latest PX4 quaternion prevents EKF2 from drifting
    //   due to a stale orientation estimate.
    //
    //   First call (no SLAM yet): sends an identity pose (origin, level) with
    //   large variances so PX4 can initialise VIO fusion safely.
    // ==========================================================================
    void evaluate_and_publish_odometry()
    {
        const rclcpp::Time now = this->get_clock()->now();

        if (!has_slam_pose_) {
            // No SLAM data received yet - send a safe identity pose to initialise EKF2 VIO
            px4_msgs::msg::VehicleOdometry vo;
            vo.timestamp        = now.nanoseconds() / 1000;
            vo.timestamp_sample = vo.timestamp;
            vo.pose_frame       = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
            vo.velocity_frame   = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;
            vo.position         = {0.0f, 0.0f, 0.0f};
            vo.q                = {0.0f, 0.0f, 0.0f, 1.0f};  // identity quaternion (no rotation)
            vo.velocity         = {0.0f, 0.0f, 0.0f};
            vo.angular_velocity = {0.0f, 0.0f, 0.0f};
            vo.position_variance    = {1.0f, 1.0f, 0.5f};  // large - untrusted initial pose
            vo.orientation_variance = {0.3f, 0.3f, 0.15f};
            vo.quality = 100;

            last_slam_pose_      = vo;
            last_slam_pose_time_ = now;
            has_slam_pose_       = true;

            external_odom_publisher_->publish(vo);

            const double yaw_deg = radToDeg(std::atan2(
                2.0 * (static_cast<double>(vo.q[3]) * vo.q[2] + static_cast<double>(vo.q[0]) * vo.q[1]),
                1.0 - 2.0 * (static_cast<double>(vo.q[1]) * vo.q[1] + static_cast<double>(vo.q[2]) * vo.q[2])));

            RCLCPP_INFO(this->get_logger(),
                "[VO INIT] ts=%s | "
                "No SLAM data yet - sending identity pose to PX4 VIO | "
                "pos_ned=[0.000 0.000 0.000] yaw=%.2fdeg quality=%d | %s",
                getTimestamp().c_str(), yaw_deg, vo.quality, lidarSummary().c_str());

        } else if ((now - last_slam_pose_time_).seconds() > 0.004) {
            // Enough time has passed since the last publish - re-send cached SLAM pose

            last_slam_pose_.timestamp        = now.nanoseconds() / 1000;
            last_slam_pose_.timestamp_sample = last_slam_pose_.timestamp;

            // Use the latest PX4 quaternion for orientation to stay in sync with autopilot heading
            const bool using_px4_quat = has_px4_q_;
            if (using_px4_quat) {
                last_slam_pose_.q = latest_px4_q_;
            }

            last_slam_pose_time_ = now;
            external_odom_publisher_->publish(last_slam_pose_);

            const auto& q = last_slam_pose_.q;
            const double yaw_deg = radToDeg(std::atan2(
                2.0 * (static_cast<double>(q[3]) * q[2] + static_cast<double>(q[0]) * q[1]),
                1.0 - 2.0 * (static_cast<double>(q[1]) * q[1] + static_cast<double>(q[2]) * q[2])));

            // DEBUG level - fires at ~200 Hz, use RCLCPP_INFO only when debugging timing
            RCLCPP_DEBUG(this->get_logger(),
                "[VO REPUB] ts=%s | "
                "Re-publishing cached SLAM pose | quat_src=%s | "
                "pos_ned=[%.3f %.3f %.3f] yaw=%.2fdeg quality=%d | %s",
                getTimestamp().c_str(),
                using_px4_quat ? "px4_latest" : "slam_cached",
                last_slam_pose_.position[0], last_slam_pose_.position[1], last_slam_pose_.position[2],
                yaw_deg, last_slam_pose_.quality,
                lidarSummary().c_str());
        }
    }

    // ==========================================================================
    // Frame offset initialisation (reserved for future SLAM<->PX4 alignment)
    //
    // Computes the rigid-body offset between the PX4 and SLAM reference frames.
    // Call this once after both a valid PX4 pose and a valid SLAM pose are
    // available to align the two coordinate origins.
    // ==========================================================================
    void initializeOffset(const Eigen::Vector3d&    px4_pos,  const Eigen::Quaterniond& px4_q,
                           const Eigen::Vector3d&    slam_pos, const Eigen::Quaterniond& slam_q)
    {
        offset_translation_ = slam_pos - px4_pos;
        offset_rotation_    = slam_q * px4_q.inverse();
        offset_initialized_ = true;

        RCLCPP_INFO(this->get_logger(),
            "[OFFSET INIT] Frame alignment computed | "
            "translation: dx=%.3f dy=%.3f dz=%.3f | "
            "rotation yaw=%.2fdeg",
            offset_translation_.x(), offset_translation_.y(), offset_translation_.z(),
            radToDeg(yawFromQuatRad(offset_rotation_)));
    }

    // ==========================================================================
    // Member variables
    // ==========================================================================

    // --- Configurable frame IDs and topic names ---
    std::string base_frame_id_;
    std::string scan_child_frame_id_;
    std::string odom_frame_id_;
    std::string scan_subscribe_topic_;
    std::string scan_publish_topic_;
    std::string odom_subscribe_topic_;
    std::string odom_publish_topic_;
    std::string pose_subscribe_topic_;
    std::string external_odom_publish_topic_;
    double      last_addition_;

    // --- ROS subscribers ---
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr             odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;

    // --- ROS publishers ---
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    scan_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr external_odom_publisher_;

    // --- TF ---
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer                                tf_buffer_;
    tf2_ros::TransformListener                     tf_listener_;

    // --- 200 Hz VIO re-publish timer ---
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    // --- Latest PX4 drone state (updated by odometryCallback) ---
    std::array<float, 4> latest_px4_q_         = {0.0f, 0.0f, 0.0f, 1.0f};  // NED/FRD quaternion
    bool                  has_px4_q_            = false;
    Eigen::Vector3d       latest_drone_pos_enu_{0.0, 0.0, 0.0};
    double                latest_drone_yaw_deg_ = 0.0;
    bool                  has_drone_pos_        = false;

    // --- Latest SLAM state (updated by posecallback) ---
    px4_msgs::msg::VehicleOdometry last_slam_pose_;
    rclcpp::Time                   last_slam_pose_time_;
    bool                            has_slam_pose_       = false;
    Eigen::Vector3d                 latest_slam_enu_pos_{0.0, 0.0, 0.0};
    double                          latest_slam_yaw_deg_ = 0.0;
    bool                             has_slam_enu_pos_    = false;

    // --- Latest LiDAR scan (updated by callback_scan) ---
    std::vector<float> latest_scan_ranges_;
    float latest_scan_angle_min_ = 0.0f;
    float latest_scan_angle_max_ = 0.0f;
    float latest_scan_range_min_ = 0.0f;
    float latest_scan_range_max_ = 0.0f;
    bool  has_latest_scan_       = false;

    // --- Frame offset for SLAM<->PX4 alignment (reserved for future use) ---
    Eigen::Vector3d    offset_translation_{0.0, 0.0, 0.0};
    Eigen::Quaterniond offset_rotation_{Eigen::Quaterniond::Identity()};
    bool                offset_initialized_;

    // --- Stored PX4 initial pose for deferred offset initialisation ---
    Eigen::Vector3d    last_px4_pos_{0.0, 0.0, 0.0};
    Eigen::Quaterniond last_px4_q_{Eigen::Quaterniond::Identity()};
    bool                last_px4_valid_ = false;
};

// ==========================================================================
// main
// ==========================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTransform>());
    rclcpp::shutdown();
    return 0;
}