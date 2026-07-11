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

using namespace px4_msgs::msg;

class PX4ScanTFBroadcaster : public rclcpp::Node {
public:
    PX4ScanTFBroadcaster() : Node("px4_scan_tf_broadcaster"),offset_initialized_(false), tf_buffer_(this->get_clock()) , tf_listener_(tf_buffer_){

        declare_parameter<std::string>("base_frame_id", "base_footprint");
        base_frame_id_ = get_parameter("base_frame_id").as_string();
        declare_parameter<std::string>("scan_child_frame_id", "laser");
        scan_child_frame_id_ = get_parameter("scan_child_frame_id").as_string();
        declare_parameter<std::string>("odom_frame_id", "odom");
        odom_frame_id_ = get_parameter("odom_frame_id").as_string();
        declare_parameter<std::string>("scan_subscribe_topic", "/scan");
        scan_subscribe_topic_ = get_parameter("scan_subscribe_topic").as_string();
        declare_parameter<std::string>("scan_publish_topic", "/scan_modified");
        scan_publish_topic_ = get_parameter("scan_publish_topic").as_string();
        declare_parameter<std::string>("odom_subscribe_topic", "/fmu/out/vehicle_odometry");
        odom_subscribe_topic_ = get_parameter("odom_subscribe_topic").as_string();
        declare_parameter<std::string>("odom_publish_topic", "/odom");
        odom_publish_topic_ = get_parameter("odom_publish_topic").as_string();
        declare_parameter<std::string>("pose_subscribe_topic", "/pose");
        pose_subscribe_topic_ = get_parameter("pose_subscribe_topic").as_string();
        declare_parameter<std::string>("external_odom_publish_topic", "/fmu/in/vehicle_visual_odometry");
        external_odom_publish_topic_ = get_parameter("external_odom_publish_topic").as_string();

        // RCLCPP_INFO(this->get_logger(), "scan_frame_id_ = %s", scan_frame_id_.c_str());

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // Subscriber to PX4 VehicleLocalPosition
        // subscription_px4_ = this->create_subscription<VehicleLocalPosition>(
        //     "/fmu/out/vehicle_local_position", qos_profile,
        //     std::bind(&PX4ScanTFBroadcaster::callback_vehicle_local_position, this, std::placeholders::_1));

        // this->declare_parameter("use_sim_time", true);  // Enable simulation time
        
        // Subscriber to LaserScan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_subscribe_topic_, qos_profile,
            std::bind(&PX4ScanTFBroadcaster::callback_scan, this, std::placeholders::_1));

        // Publisher for modified scan data (same topic as original)
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_publish_topic_, 10);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_publish_topic_, 10);
        // odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odom_rf2o", qos_profile,
        //     std::bind(&PX4ScanTFBroadcaster::odometryCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<VehicleOdometry>(
            odom_subscribe_topic_, qos_profile,
            std::bind(&PX4ScanTFBroadcaster::odometryCallback, this, std::placeholders::_1)
        );

        external_odom_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            external_odom_publish_topic_, 10);

        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_subscribe_topic_, 10,
            std::bind(&PX4ScanTFBroadcaster::posecallback, this, std::placeholders::_1));

        publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&PX4ScanTFBroadcaster::evaluate_and_publish_odometry, this));

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100), std::bind(&PX4ScanTFBroadcaster::publish_tf, this)
        // );

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize TF listener
        // tf_listener_ = tf2_ros::TransformListener(tf_buffer_, *this);

        // Start timer to check for transform and publish VO
        // tf_timer_ = this->create_wall_timer(
        // std::chrono::milliseconds(4),
        // std::bind(&PX4ScanTFBroadcaster::publish_visual_odometry_from_tf, this));
        declare_parameter<double>("last_addition", 0.02);

    }

private:
    Eigen::Vector3d offset_translation_;
    Eigen::Quaterniond offset_rotation_;
    bool offset_initialized_;

    void initializeOffset(const Eigen::Vector3d& px4_pos, const Eigen::Quaterniond& px4_q,const Eigen::Vector3d& slam_pos, const Eigen::Quaterniond& slam_q){
        // Translation offset
        offset_translation_ = slam_pos - px4_pos;

        // Rotation offset (relative quaternion)
        offset_rotation_ = slam_q * px4_q.inverse();

        offset_initialized_ = true;

        RCLCPP_INFO(this->get_logger(),
            "Offset initialized: dx=%.2f, dy=%.2f, dz=%.2f, yaw=%.2f deg",
            offset_translation_.x(), offset_translation_.y(), offset_translation_.z(),
            radToDeg(yawFromQuatRad(offset_rotation_)));
    }
    
    // NED → ENU orientation: swap X/Y, flip Z, keep W
    Eigen::Quaterniond nedToEnuOrientation(const Eigen::Quaterniond& q_ned) {
        return Eigen::Quaterniond(q_ned.w(), q_ned.y(), q_ned.x(), -q_ned.z());
    }

    // ENU baselink → aircraft orientation (PX4 convention)
    // Implemented as 180° rotation about X axis (self-inverse)
    Eigen::Quaterniond baselinkToAircraftOrientation(const Eigen::Quaterniond& q_enu) {
        Eigen::Quaterniond q_flip(0, 1, 0, 0); // 180° around X
        return q_flip * q_enu;
    }

    // ENU → NED orientation
    Eigen::Quaterniond enuToNedOrientation(const Eigen::Quaterniond& q_enu) {
        return Eigen::Quaterniond(q_enu.w(), q_enu.y(), q_enu.x(), -q_enu.z());
    }

    // Aircraft → Baselink orientation (self-inverse flip around X)
    Eigen::Quaterniond aircraftToBaselinkOrientation(const Eigen::Quaterniond& q_aircraft) {
        Eigen::Quaterniond q_flip(0, 1, 0, 0); // 180° rotation around X
        return q_flip * q_aircraft;
    }

    // Convert PX4 msg->q (qx,qy,qz,qw) to Eigen (w,x,y,z)
    Eigen::Quaterniond arrayToEigenQuat(const std::array<float,4>& q) {
        return Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
    }

    // Compute yaw (rotation about Z) from quaternion (ENU convention)
    double yawFromQuatRad(const Eigen::Quaterniond& q) {
        const double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
        const double num = 2.0 * (qw * qz + qx * qy);
        const double den = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(num, den);
    }

    double radToDeg(double r) { return r * 180.0 / M_PI; }

    // Convert Eigen quaternion to ROS 2 message
    geometry_msgs::msg::Quaternion eigenToRosQuat(const Eigen::Quaterniond& q) {
        geometry_msgs::msg::Quaternion ros_q;
        ros_q.x = q.x(); ros_q.y = q.y(); ros_q.z = q.z(); ros_q.w = q.w();
        return ros_q;
    }

    void odometryCallback(const VehicleOdometry::SharedPtr msg) {
        nav_msgs::msg::Odometry publish_data;
        publish_data.header.stamp = this->get_clock()->now();
        publish_data.header.frame_id = odom_frame_id_;
        publish_data.child_frame_id = base_frame_id_;

        // Get current system time
        auto tnow = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()) % 1000;

        std::time_t now_c = std::chrono::system_clock::to_time_t(tnow);
        std::tm local_tm = *std::localtime(&now_c);

        // Format into YYYY-MM-DD HH:MM:SS.mmm
        std::ostringstream time_stream;
        time_stream << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                    << '.' << std::setfill('0') << std::setw(3) << ms.count();

        RCLCPP_INFO(this->get_logger(), "vehicle_odometry at %s",time_stream.str().c_str());

        // Convert quaternion (NED → ENU)
        Eigen::Quaterniond q = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
        // // Eigen::Quaterniond q_ned(msg->q[3], msg->q[0], msg->q[1], msg->q[2]);
        // // Eigen::Quaterniond q_enu = nedToEnuOrientation(q_ned);
        // // Eigen::Quaterniond enu_base_link_q = baselinkToAircraftOrientation(q_enu);
        // Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::ned_to_enu_orientation(q);
        // Eigen::Quaterniond q_enu_aircraft = px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(q_ned);

        // --- Position conversion (NED → ENU) ---
        Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
        Eigen::Vector3d pos_enu(pos_ned.y(), pos_ned.x(), -pos_ned.z());

        // --- Orientation conversion (NED/FRD → ENU/FLU) ---
        // Eigen::Quaterniond q_px4(msg->q[3], msg->q[0], msg->q[1], msg->q[2]); // PX4 order: w,x,y,z
        // Eigen::Quaterniond q_ned_enu( q_px4.w(), q_px4.y(), q_px4.x(), -q_px4.z() ); // NED→ENU
        Eigen::Quaterniond q_ned_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(q);
        Eigen::Quaterniond q_frd_flu(0, 1, 0, 0); // 180° about X (FRD→FLU)
        // Eigen::Quaterniond q_ned = q_frd_flu * q_ned_enu;
        Eigen::Quaterniond q_ros = px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(q_ned_enu);
        q_ros.normalize();

        // // --- Sign continuity fix --- 
        // static Eigen::Quaterniond last_q_ros(0,0,0,0); 
        // if (last_q_ros.norm() > 0.0 && q_ros.dot(last_q_ros) < 0.0) { 
        //     q_ros.coeffs() *= -1.0; 
        // } 
        // last_q_ros = q_ros;

        // --- Enforce quaternion continuity (ignore sign-only flips) ---
        // if (prev_valid) {
        //     if (q_ros.dot(prev_q_ros) < 0.0) {
        //         q_ros.coeffs() *= -1.0;  // flip sign to match previous
        //     }
        // }

        // // --- Compute yaw from quaternion ---
        // double yaw_rad = yawFromQuatRad(q_ros);

        // // --- Spike detection logic ---
        // if (prev_valid) {
        //     double delta_yaw = yaw_rad - prev_yaw_rad;
        //     // unwrap to [-pi, pi]
        //     if (delta_yaw > M_PI) delta_yaw -= 2*M_PI;
        //     if (delta_yaw < -M_PI) delta_yaw += 2*M_PI;

        //     double rounded_yaw = std::round(std::fabs(yaw_rad) * 100.0) / 100.0;

        //     if (rounded_yaw != 1.57  && std::fabs(prev_yaw_rad) <= 2.21 && std::fabs(delta_yaw) > (40.0 * M_PI / 180.0)) {
        //         // Spike detected → extrapolate using previous delta
        //         double new_yaw = prev_yaw_rad + prev_delta_yaw;

        //         // Rebuild quaternion with corrected yaw (keep roll/pitch from q_ros)
        //         Eigen::Vector3d euler = q_ros.toRotationMatrix().eulerAngles(0,1,2);
        //         euler[2] = new_yaw; // replace yaw
        //         q_ros = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
        //             * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
        //             * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
        //         q_ros.normalize();

        //         yaw_rad = new_yaw;
        //         delta_yaw = prev_delta_yaw;

        //         // 🔔 Logger for spike detection
        //         RCLCPP_WARN(this->get_logger(),
        //             "Yaw spike detected! Raw delta=%.2f deg, corrected yaw=%.2f deg",
        //             radToDeg(delta_yaw), radToDeg(new_yaw));

        //     }

        //     prev_delta_yaw = delta_yaw;
        // }

        // prev_yaw_rad = yaw_rad;
        // prev_q_ros = q_ros;
        // prev_valid = true;

        // 1) Raw NED quaternion from PX4
        // Eigen::Quaterniond q_ned = arrayToEigenQuat(msg->q);

        // 2) Convert NED → ENU
        // Eigen::Quaterniond q_enu = nedToEnuOrientation(q_ned);

        // 3) Apply baselink → aircraft orientation in ENU
        // Eigen::Quaterniond q_enu_aircraft = baselinkToAircraftOrientation(q_enu);

        // 4) Compute yaw before (from NED) and after (from ENU aircraft)
        double yaw_ned_rad = yawFromQuatRad(q);              // interpret directly (for comparison)
        double yaw_enu_aircraft_rad = yawFromQuatRad(q_ros);

        double yaw_ned_deg = radToDeg(yaw_ned_rad);
        double yaw_enu_aircraft_deg = radToDeg(yaw_enu_aircraft_rad);

        // Convert position (NED → ENU)
        // Eigen::Vector3d position(msg->position[0], msg->position[1], msg->position[2]);
        // Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(position);

        // // Convert velocity (NED → ENU)
        // Eigen::Vector3d velocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        // Eigen::Vector3d enu_velocity = px4_ros_com::frame_transforms::ned_to_enu_local_frame(velocity);

        // ✅ If offset is initialized, apply it
        // if (offset_initialized_) {
        //     enu_position += offset_translation_;
        //     q_enu_aircraft = offset_rotation_ * q_enu_aircraft;
        //     q_enu_aircraft.normalize();

        //     // enforce quaternion sign consistency
        //     if (q_enu_aircraft.w() < 0) {
        //         q_enu_aircraft.coeffs() *= -1;
        //     }
        // } else {
        //     // ✅ If offset not initialized, store last PX4 values for later use
        //     last_px4_pos_ = enu_position;
        //     last_px4_q_   = q_enu_aircraft;
        //     last_px4_valid_ = true;

        //     RCLCPP_INFO(this->get_logger(),
        //         "Stored PX4 initial pose for offset init (x=%.2f, y=%.2f, z=%.2f, yaw=%.2f deg)",
        //         enu_position.x(), enu_position.y(), enu_position.z(),
        //         radToDeg(yawFromQuatRad(q_enu_aircraft)));
        // }
    
        // Apply conversions
        // publish_data.pose.pose.position.x = enu_position.x();
        // publish_data.pose.pose.position.y = enu_position.y();
        // publish_data.pose.pose.position.z = enu_position.z();
        // publish_data.pose.pose.orientation.x = q_enu_aircraft.x();
        // publish_data.pose.pose.orientation.y = q_enu_aircraft.y();
        // publish_data.pose.pose.orientation.z = q_enu_aircraft.z();
        // publish_data.pose.pose.orientation.w = q_enu_aircraft.w();
        // publish_data.twist.twist.linear.x = enu_velocity.x();
        // publish_data.twist.twist.linear.y = enu_velocity.y();
        // publish_data.twist.twist.linear.z = enu_velocity.z();

        // // Convert angular velocity (NED → ENU) with PX4 convention
        // publish_data.twist.twist.angular.x = msg->angular_velocity[0];  // Roll speed
        // publish_data.twist.twist.angular.y = -msg->angular_velocity[1]; // Negate pitch speed
        // publish_data.twist.twist.angular.z = -msg->angular_velocity[2]; // Negate yaw speed

        // // Convert Eigen Quaternion to ROS 2 Quaternion
        // geometry_msgs::msg::Quaternion ros_quat;
        // ros_quat.x = enu_q.x();
        // ros_quat.y = enu_q.y();
        // ros_quat.z = enu_q.z();
        // ros_quat.w = enu_q.w();

        // // Apply baselink → aircraft convention transformation
        // publish_data.pose.pose.orientation =
        //     px4_ros_com::frame_transforms::ned_to_enu_orientation(
        //         px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(ros_quat));


        // --- Fill ROS odometry ---
        publish_data.pose.pose.position.x = pos_enu.x();
        publish_data.pose.pose.position.y = pos_enu.y();
        publish_data.pose.pose.position.z = pos_enu.z();
        publish_data.pose.pose.orientation.x = q_ros.x();
        publish_data.pose.pose.orientation.y = q_ros.y();
        publish_data.pose.pose.orientation.z = q_ros.z();
        publish_data.pose.pose.orientation.w = q_ros.w();

        // Velocity conversion (NED → ENU)
        Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        Eigen::Vector3d vel_enu(vel_ned.y(), vel_ned.x(), -vel_ned.z());
        publish_data.twist.twist.linear.x = vel_enu.x();
        publish_data.twist.twist.linear.y = vel_enu.y();
        publish_data.twist.twist.linear.z = vel_enu.z();

        odom_publisher_->publish(publish_data);

        // Publish transformed odometry
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = this->get_clock()->now();
        odom_tf.header.frame_id = odom_frame_id_;
        odom_tf.child_frame_id = base_frame_id_;

        odom_tf.transform.translation.x = publish_data.pose.pose.position.x;
        odom_tf.transform.translation.y = publish_data.pose.pose.position.y;
        odom_tf.transform.translation.z = publish_data.pose.pose.position.z;

        odom_tf.transform.rotation = publish_data.pose.pose.orientation;

        tf_broadcaster_->sendTransform(odom_tf);
        
        // --- Calculate yaw from raw quaternion msg->q ---
        double qx_raw = msg->q[0];
        double qy_raw = msg->q[1];
        double qz_raw = msg->q[2];
        double qw_raw = msg->q[3];
        // double yaw_rad_raw = std::atan2(2.0 * (qw_raw * qz_raw + qx_raw * qy_raw),
        //                                 1.0 - 2.0 * (qy_raw * qy_raw + qz_raw * qz_raw));
        // double yaw_deg_raw = yaw_rad_raw * 180.0 / M_PI;

        // --- Calculate yaw from converted ENU quaternion publish_data.pose.pose.orientation ---
        double qx_conv = publish_data.pose.pose.orientation.x;
        double qy_conv = publish_data.pose.pose.orientation.y;
        double qz_conv = publish_data.pose.pose.orientation.z;
        double qw_conv = publish_data.pose.pose.orientation.w;
        // double yaw_rad_conv = std::atan2(2.0 * (qw_conv * qz_conv + qx_conv * qy_conv),
        //                                 1.0 - 2.0 * (qy_conv * qy_conv + qz_conv * qz_conv));
        // double yaw_deg_conv = yaw_rad_conv * 180.0 / M_PI;

        // ✅ Print both raw quaternion + yaw, and converted quaternion + yaw
        RCLCPP_INFO(this->get_logger(),
            "Raw PX4 quaternion (x=%.2f, y=%.2f, z=%.2f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f), yaw=%.2f deg",
            pos_ned.x(), pos_ned.y(), pos_ned.z(), qx_raw, qy_raw, qz_raw, qw_raw, yaw_ned_deg);

        RCLCPP_INFO(this->get_logger(),
            "Converted ENU odometry (x=%.2f, y=%.2f, z=%.2f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f, yaw=%.2f deg)",
            publish_data.pose.pose.position.x, publish_data.pose.pose.position.y, publish_data.pose.pose.position.z,
            qx_conv, qy_conv, qz_conv, qw_conv, yaw_enu_aircraft_deg);
    }

    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto modified_scan = *msg;  // Copy the original message

        modified_scan.header.stamp = this->get_clock()->now();
        // Update the frame_id to match the desired transformation
        modified_scan.header.frame_id = scan_child_frame_id_;  // Mapping scan data to base_link

        // Publish modified scan data back on the same topic
        scan_publisher_->publish(modified_scan);

        RCLCPP_INFO(this->get_logger(), "Republished scan data with updated frame_id on /scan");

        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = base_frame_id_;
        transform.child_frame_id = scan_child_frame_id_;

        // Adjust position for Slamtech C1 LiDAR on quadcopter
        transform.transform.translation.x = 0.0;  // Modify based on actual mounting
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        // Set orientation (forward-facing LiDAR)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform);
    }

    void posecallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        rclcpp::Time now = this->get_clock()->now();

        // --- Position conversion (ENU → NED) ---
        Eigen::Vector3d pos_enu(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
        Eigen::Vector3d pos_ned(pos_enu.y(), pos_enu.x(), -0.0f);

        // --- Orientation conversion (ENU/FLU → NED/FRD) ---
        Eigen::Quaterniond q_enu(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);
        // Eigen::Quaterniond q_enu_ned(q_enu.w(), q_enu.y(), q_enu.x(), -q_enu.z()); // ENU→NED
        Eigen::Quaterniond q_enu_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        Eigen::Quaterniond q_flu_frd(0, 1, 0, 0); // 180° about X (FLU→FRD)
        // Eigen::Quaterniond q_ned = q_flu_frd * q_enu_ned;
        Eigen::Quaterniond q_px4 = px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(q_enu_ned);
        q_px4.normalize();
        
        // // --- Sign continuity fix --- 
        // static Eigen::Quaterniond last_q_px4(0,0,0,0); 
        // if (last_q_px4.norm() > 0.0 && q_px4.dot(last_q_px4) < 0.0) { 
        //     q_px4.coeffs() *= -1.0; 
        // } 
        // last_q_px4 = q_px4;

        // tf2::Quaternion q_enu(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w); 
        
        // // ENU -> NED rotation: roll=π, pitch=0, yaw=π/2 
        // tf2::Quaternion R_enu2ned; 
        // R_enu2ned.setRPY(M_PI, 0.0, M_PI/2.0); 
        
        // // Apply transform 
        // tf2::Quaternion q_px4 = R_enu2ned * q_enu; 
        // q_px4.normalize();
        
        // If PX4 odometry already received and offset not yet initialized → calculate
        // if (!offset_initialized_ && last_px4_valid_) {
        //     initializeOffset(last_px4_pos_, last_px4_q_, pos_enu, q_enu);
        // }

        // 2. Convert ENU to NED
        // Eigen::Vector3d pos_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(pos_enu);
        // geometry_msgs::msg::Quaternion q_ros = msg->pose.pose.orientation;

        // std::array<float, 4> q_array = {
        //     static_cast<float>(q_ros.w),
        //     static_cast<float>(q_ros.x),
        //     static_cast<float>(q_ros.y),
        //     static_cast<float>(q_ros.z)
        // };

        // Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(q_array);
        // Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        // Eigen::Quaterniond q_baselink = px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(q_ned);

        // q_baselink.normalize();
        // if (q_baselink.w() < 0) {
        //     q_baselink.coeffs() *= -1;   // enforce consistent sign
        // }
        // Step 1: ENU → NED
        // Eigen::Quaterniond q_enu(q_ros.w, q_ros.x, q_ros.y, q_ros.z);

        // Eigen::Quaterniond q_ned = enuToNedOrientation(q_enu);

        // Step 2: Aircraft → Baselink (applied to ENU quaternion if needed)
        // Eigen::Quaterniond q_baselink = aircraftToBaselinkOrientation(q_ned);

        // Step 3: Compute yaw angles
        double yaw_enu_deg = radToDeg(yawFromQuatRad(q_enu));
        double yaw_ned_deg = radToDeg(yawFromQuatRad(q_px4));
        // double yaw_baselink_deg = radToDeg(yawFromQuatRad(q_baselink));

        // 3. Create PX4 VIO message
        px4_msgs::msg::VehicleOdometry vo;
        vo.timestamp = now.nanoseconds() / 1000;
        vo.timestamp_sample = vo.timestamp;

        vo.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        vo.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;

        vo.position = {
            static_cast<float>(pos_ned.x()),
            static_cast<float>(pos_ned.y()),
            static_cast<float>(pos_ned.z())
        };

        vo.q = {
            static_cast<float>(q_px4.x()),
            static_cast<float>(q_px4.y()),
            static_cast<float>(q_px4.z()),
            static_cast<float>(q_px4.w())
        };

        // SLAM provides pose only, so publish explicit zero velocity and angular velocity
        // vo.velocity = {0.0f, 0.0f, 0.0f};
        // vo.angular_velocity = {0.0f, 0.0f, 0.0f};

        // 4. Use diagonal of covariance for variance
        vo.position_variance = {
            static_cast<float>(msg->pose.covariance[0]),
            static_cast<float>(msg->pose.covariance[7]),
            static_cast<float>(25.0f)
        };
        vo.orientation_variance = {
            static_cast<float>(msg->pose.covariance[21]),
            static_cast<float>(msg->pose.covariance[28]),
            static_cast<float>(msg->pose.covariance[35])
        };

        // px4_msgs::msg::VehicleOdometry vo;
        // vo.timestamp = now.nanoseconds() / 1000;
        // vo.timestamp_sample = vo.timestamp;

        // Eigen::Vector3d pos_enu(msg->pose.pose.position.x,
        //                     msg->pose.pose.position.y,
        //                     msg->pose.pose.position.z);

        // Eigen::Quaterniond q_enu(
        //     msg->pose.pose.orientation.w,
        //     msg->pose.pose.orientation.x,
        //     msg->pose.pose.orientation.y,
        //     msg->pose.pose.orientation.z);

        // // 🔄 Position: ENU → NED
        // Eigen::Vector3d pos_ned = px4_ros_com::frame_transforms::transform_static_frame(
        //     pos_enu, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

        // // 🔁 Orientation: ENU → NED
        // Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::transform_orientation(
        //     q_enu, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

        // // 🧭 Fill VehicleOdometry
        // vo.position[0] = pos_ned.x();
        // vo.position[1] = pos_ned.y();
        // vo.position[2] = pos_ned.z();

        // vo.q[0] = 0.0;
        // vo.q[1] = q_ned.y();
        // vo.q[2] = 0.0;
        // vo.q[3] = 0.0;


        // vo.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        // // vo.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        // vo.position_variance = {
        //     static_cast<float>(msg->pose.covariance[0]),
        //     static_cast<float>(msg->pose.covariance[7]),
        //     static_cast<float>(msg->pose.covariance[14])
        // };
        // vo.orientation_variance = {
        //     static_cast<float>(msg->pose.covariance[21]),
        //     static_cast<float>(msg->pose.covariance[28]),
        //     static_cast<float>(msg->pose.covariance[35])
        // };

        vo.quality = 100;

        last_slam_pose_ = vo;
        last_slam_pose_time_ = this->get_clock()->now();
        has_slam_pose_ = true;

        // 5. Publish to PX4
        external_odom_publisher_->publish(vo);

        // Get current system time
        auto tnow = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()) % 1000;

        std::time_t now_c = std::chrono::system_clock::to_time_t(tnow);
        std::tm local_tm = *std::localtime(&now_c);

        // Format into YYYY-MM-DD HH:MM:SS.mmm
        std::ostringstream time_stream;
        time_stream << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                    << '.' << std::setfill('0') << std::setw(3) << ms.count();

        // Log with timestamp including milliseconds
        RCLCPP_INFO(this->get_logger(), "Pose -> vehicle_visual_odometry at %s",time_stream.str().c_str());
        // --- Calculate yaw from raw quaternion msg->q ---
        double qx_raw = q_enu.x();
        double qy_raw = q_enu.y();
        double qz_raw = q_enu.z();
        double qw_raw = q_enu.w();
        // double yaw_rad_raw = std::atan2(2.0 * (qw_raw * qz_raw + qx_raw * qy_raw),
        //                                 1.0 - 2.0 * (qy_raw * qy_raw + qz_raw * qz_raw));
        // double yaw_deg_raw = yaw_rad_raw * 180.0 / M_PI;

        // --- Calculate yaw from converted ENU quaternion publish_data.pose.pose.orientation ---
        // double qx_conv = vo.q[0];
        // double qy_conv = vo.q[1];
        // double qz_conv = vo.q[2];
        // double qw_conv = vo.q[3];
        // double yaw_rad_conv = std::atan2(2.0 * (qw_conv * qz_conv + qx_conv * qy_conv),
        //                                 1.0 - 2.0 * (qy_conv * qy_conv + qz_conv * qz_conv));
        // double yaw_deg_conv = yaw_rad_conv * 180.0 / M_PI;

        // ✅ Print both raw quaternion + yaw, and converted quaternion + yaw
        RCLCPP_INFO(this->get_logger(),
            "ENU SLAM Input (x=%.2f, y=%.2f, z=%.2f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f, yaw=%.2f deg)",
            pos_enu.x(), pos_enu.y(), pos_enu.z(), qx_raw, qy_raw, qz_raw, qw_raw, yaw_enu_deg);

        RCLCPP_INFO(this->get_logger(),
            "NED PX4 VO Published (pos_ned [%.4f, %.4f, %.4f], q [%.4f, %.4f, %.4f, %.4f], yaw=%.2f deg, quality=%d)",
            vo.position[0], vo.position[1], vo.position[2], vo.q[0], vo.q[1], vo.q[2], vo.q[3], yaw_ned_deg, vo.quality);
    }

    // Convert ENU/FLU quaternion to NED/FRD quaternion
    Eigen::Quaternionf enu_flu_to_ned_frd(const Eigen::Quaternionf& q_enu_flu)
    {
        // ENU -> NED rotation (swap X/Y, flip Z)
        Eigen::Matrix3f R_enu_to_ned;
        R_enu_to_ned << 0, 1, 0,
                        1, 0, 0,
                        0, 0,-1;

        // FLU -> FRD rotation (flip Y and Z)
        Eigen::Matrix3f R_flu_to_frd;
        R_flu_to_frd << 1, 0, 0,
                        0,-1, 0,
                        0, 0,-1;

        // Convert fixed rotations to quaternions
        Eigen::Quaternionf q_world(R_enu_to_ned);
        Eigen::Quaternionf q_body(R_flu_to_frd);

        // Apply: q_ned_frd = q_world * q_enu_flu * q_body
        Eigen::Quaternionf q_ned_frd = q_world * q_enu_flu * q_body;
        q_ned_frd.normalize();

        return q_ned_frd;
    }


    void evaluate_and_publish_odometry() {

        rclcpp::Time now = this->get_clock()->now();

        if (!has_slam_pose_) {
            // 3. Create PX4 VIO message
            px4_msgs::msg::VehicleOdometry vo;
            vo.timestamp = now.nanoseconds() / 1000;
            vo.timestamp_sample = vo.timestamp;

            vo.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
            vo.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;

            vo.position = {
                static_cast<float>(0.0f),
                static_cast<float>(0.0f),
                static_cast<float>(0.0f)
            };

            vo.q = {
                static_cast<float>(0.0f),
                static_cast<float>(0.0f),
                static_cast<float>(0.0f),
                static_cast<float>(1.0f)
            };

            // Original ENU/FLU quaternion (from SLAM/VIO)
            // Eigen::Quaternionf q_enu_flu(vo.q[3], vo.q[0], vo.q[1], vo.q[2]); // w,x,y,z

            // // Convert to NED/FRD
            // Eigen::Quaternionf q_ned_frd = enu_flu_to_ned_frd(q_enu_flu);

            // // Assign back to PX4 message
            // vo.q = {q_ned_frd.x(), q_ned_frd.y(), q_ned_frd.z(), q_ned_frd.w()};

            // Optional: Set zero velocity (or add estimate if needed)
            vo.velocity = {0.0f, 0.0f, 0.0f};
            vo.angular_velocity = {0.0f, 0.0f, 0.0f};

            // 4. Use diagonal of covariance for variance
            vo.position_variance = {
                static_cast<float>(1.0f),
                static_cast<float>(1.0f),
                static_cast<float>(0.5f)
            };

            vo.orientation_variance = {
                static_cast<float>(0.3f),
                static_cast<float>(0.3f),
                static_cast<float>(0.15f)
            };

            vo.quality = 100;

            last_slam_pose_ = vo;
            last_slam_pose_time_ = this->get_clock()->now();
            has_slam_pose_ = true;

            auto q = vo.q;
            double yaw = std::atan2(2.0 * (q[3]*q[2] + q[0]*q[1]),
                                    1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]));
            RCLCPP_INFO(this->get_logger(), "VO yaw sent: %.2f deg , %.2d quality", yaw * 180.0 / M_PI, vo.quality);

            // 5. Publish to PX4
            external_odom_publisher_->publish(vo);
            // Get current system time
            auto tnow = std::chrono::system_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()) % 1000;

            std::time_t now_c = std::chrono::system_clock::to_time_t(tnow);
            std::tm local_tm = *std::localtime(&now_c);

            // Format into YYYY-MM-DD HH:MM:SS.mmm
            std::ostringstream time_stream;
            time_stream << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                        << '.' << std::setfill('0') << std::setw(3) << ms.count();

            // Log with timestamp including milliseconds
            RCLCPP_INFO(this->get_logger(), "First time -> vehicle_visual_odometryat %s",time_stream.str().c_str());
        }

        // Publish only if it's been more than 500ms since last publish
        else if ((now - last_slam_pose_time_).seconds() > 0.004) {
            // slam_pose_callback(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(last_slam_pose_));

            last_slam_pose_.timestamp = now.nanoseconds() / 1000;
            last_slam_pose_.timestamp_sample = last_slam_pose_.timestamp;
            last_slam_pose_.position[0] = last_slam_pose_.position[0]+last_addition;

            last_addition = -1*last_addition;

            last_slam_pose_time_ = this->get_clock()->now();
            has_slam_pose_ = true;

            // 5. Publish to PX4
            external_odom_publisher_->publish(last_slam_pose_);
            auto q = last_slam_pose_.q;
            double yaw = std::atan2(2.0 * (q[3]*q[2] + q[0]*q[1]),
                                    1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]));
            RCLCPP_INFO(this->get_logger(), "VO yaw sent: %.2f deg", yaw * 180.0 / M_PI);
            // Get current system time
            auto tnow = std::chrono::system_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()) % 1000;

            std::time_t now_c = std::chrono::system_clock::to_time_t(tnow);
            std::tm local_tm = *std::localtime(&now_c);

            // Format into YYYY-MM-DD HH:MM:SS.mmm
            std::ostringstream time_stream;
            time_stream << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                        << '.' << std::setfill('0') << std::setw(3) << ms.count();

            // Log with timestamp including milliseconds
            RCLCPP_INFO(this->get_logger(), "Same -> vehicle_visual_odometry %s",time_stream.str().c_str());
        }
    }

    // rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_px4_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_subscription_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    // rclcpp::TimerBase::SharedPtr tf_timer_;
    geometry_msgs::msg::TransformStamped prev_tf_map_base_;
    Eigen::Vector3d prev_pos_;
    Eigen::Quaterniond prev_q_enu_;
    rclcpp::Time prev_time_;
    bool has_prev_ = false;

    rclcpp::TimerBase::SharedPtr publisher_timer_;
    // geometry_msgs::msg::PoseWithCovarianceStamped last_slam_pose_;
    px4_msgs::msg::VehicleOdometry last_slam_pose_;
    rclcpp::Time last_slam_pose_time_;
    bool has_slam_pose_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr external_odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string base_frame_id_;
    std::string scan_child_frame_id_;
    std::string odom_frame_id_;
    std::string scan_subscribe_topic_;
    std::string scan_publish_topic_;
    std::string odom_subscribe_topic_;
    std::string odom_publish_topic_;
    std::string pose_subscribe_topic_;
    std::string external_odom_publish_topic_;
    double last_addition;

    // Store last PX4 pose for offset init
    Eigen::Vector3d last_px4_pos_;
    Eigen::Quaterniond last_px4_q_;
    bool last_px4_valid_ = false;

    Eigen::Quaterniond prev_q_ros;
    double prev_yaw_rad = 0.0;
    double prev_delta_yaw = 0.0;
    bool prev_valid = false;

    // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4ScanTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}