#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <px4_ros_com/frame_transforms.h>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace px4_msgs::msg;

class PX4ScanTFBroadcaster : public rclcpp::Node {
public:
    PX4ScanTFBroadcaster() : Node("px4_scan_tf_broadcaster") {

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

        odom_subscription_ = this->create_subscription<VehicleOdometry>(
            odom_subscribe_topic_, qos_profile,
            std::bind(&PX4ScanTFBroadcaster::odometryCallback, this, std::placeholders::_1)
        );

        external_odom_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            external_odom_publish_topic_, 10);

        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_subscribe_topic_, 10,
            std::bind(&PX4ScanTFBroadcaster::posecallback, this, std::placeholders::_1));

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100), std::bind(&PX4ScanTFBroadcaster::publish_tf, this)
        // );

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void odometryCallback(const VehicleOdometry::SharedPtr msg) {
        nav_msgs::msg::Odometry publish_data;
        publish_data.header.stamp = this->get_clock()->now();
        publish_data.header.frame_id = odom_frame_id_;
        publish_data.child_frame_id = base_frame_id_;

        // Convert quaternion (NED → ENU)
        Eigen::Quaterniond q = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
        Eigen::Quaterniond enu_q = px4_ros_com::frame_transforms::ned_to_enu_orientation(q);
        Eigen::Quaterniond enu_base_link_q = px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(enu_q);

        // Convert position (NED → ENU)
        Eigen::Vector3d position(msg->position[0], msg->position[1], msg->position[2]);
        Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(position);

        // Convert velocity (NED → ENU)
        Eigen::Vector3d velocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        Eigen::Vector3d enu_velocity = px4_ros_com::frame_transforms::ned_to_enu_local_frame(velocity);

        // Apply conversions
        publish_data.pose.pose.position.x = enu_position.x();
        publish_data.pose.pose.position.y = enu_position.y();
        publish_data.pose.pose.position.z = enu_position.z();
        publish_data.pose.pose.orientation.x = enu_base_link_q.x();
        publish_data.pose.pose.orientation.y = enu_base_link_q.y();
        publish_data.pose.pose.orientation.z = enu_base_link_q.z();
        publish_data.pose.pose.orientation.w = enu_base_link_q.w();
        publish_data.twist.twist.linear.x = enu_velocity.x();
        publish_data.twist.twist.linear.y = enu_velocity.y();
        publish_data.twist.twist.linear.z = enu_velocity.z();

        // Convert angular velocity (NED → ENU) with PX4 convention
        publish_data.twist.twist.angular.x = msg->angular_velocity[0];  // Roll speed
        publish_data.twist.twist.angular.y = -msg->angular_velocity[1]; // Negate pitch speed
        publish_data.twist.twist.angular.z = -msg->angular_velocity[2]; // Negate yaw speed

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


        RCLCPP_INFO(this->get_logger(), "Published ENU odometry data (x=%.2f, y=%.2f, z=%.2f)",
                    publish_data.pose.pose.position.x, publish_data.pose.pose.position.y, publish_data.pose.pose.position.z);
    }

    // void callback_vehicle_local_position(const VehicleLocalPosition::SharedPtr msg) {
    //     geometry_msgs::msg::TransformStamped tf_msg;

    //     tf_msg.header.stamp = this->get_clock()->now();
    //     tf_msg.header.frame_id = "odom";   // Global reference frame
    //     tf_msg.child_frame_id = "base_footprint"; // PX4 vehicle frame

    //     Eigen::Vector3d position(msg->x, msg->y, msg->z);
    //     Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(position);

    //     tf_msg.transform.translation.x = enu_position.x();
    //     tf_msg.transform.translation.y = enu_position.y();
    //     tf_msg.transform.translation.z = enu_position.z();

    //     // PX4 does not provide orientation, setting identity (no rotation)
    //     tf_msg.transform.rotation.w = 1.0;
    //     tf_msg.transform.rotation.x = 0.0;
    //     tf_msg.transform.rotation.y = 0.0;
    //     tf_msg.transform.rotation.z = 0.0;

    //     tf_broadcaster_->sendTransform(tf_msg);
    //     RCLCPP_INFO(this->get_logger(), "Published TF: odom -> base_footprint (x=%.2f, y=%.2f, z=%.2f)", msg->x, msg->y, msg->z);
    // }

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
        transform.transform.translation.x = 0.2;  // Modify based on actual mounting
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.1;

        // Set orientation (forward-facing LiDAR)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform);
    }

    void posecallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        px4_msgs::msg::VehicleOdometry vo_msg;

        vo_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Convert pose from ENU to NED
        Eigen::Vector3d enu_pos(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

        vo_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        Eigen::Vector3d ned_pos = px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu_pos);
        vo_msg.position[0] = ned_pos.x();
        vo_msg.position[1] = ned_pos.y();
        vo_msg.position[2] = ned_pos.z();

        // Orientation
        Eigen::Quaterniond q_enu(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

        auto q_ned = px4_ros_com::frame_transforms::enu_to_ned_orientation(q_enu);
        vo_msg.q[0] = q_ned.x();
        vo_msg.q[1] = q_ned.y();
        vo_msg.q[2] = q_ned.z();
        vo_msg.q[3] = q_ned.w();

        // Optional: set pose covariance (first 21 elements)
        // for (int i = 0; i < 21; ++i)
        // vo_msg.pose_covariance[i] = static_cast<float>(msg->pose.covariance[i]);
        vo_msg.position_variance[0] = msg->pose.covariance[0];   // x
        vo_msg.position_variance[1] = msg->pose.covariance[7];   // y
        vo_msg.position_variance[2] = msg->pose.covariance[14];  // z

        vo_msg.orientation_variance[0] = msg->pose.covariance[21]; // roll
        vo_msg.orientation_variance[1] = msg->pose.covariance[28]; // pitch
        vo_msg.orientation_variance[2] = msg->pose.covariance[35]; // yaw


        // You can set velocity, angular_velocity, or reset them to 0
        vo_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;


        external_odom_publisher_->publish(vo_msg);
    }
    // void publish_tf() {
    //     geometry_msgs::msg::TransformStamped transform;

    //     transform.header.stamp = this->get_clock()->now();
    //     transform.header.frame_id = "base_footprint";
    //     transform.child_frame_id = "x500_lidar_2d_0/link/lidar_2d_v2";

    //     // Adjust position for Slamtech C1 LiDAR on quadcopter
    //     transform.transform.translation.x = 0.2;  // Modify based on actual mounting
    //     transform.transform.translation.y = 0.0;
    //     transform.transform.translation.z = 0.1;

    //     // Set orientation (forward-facing LiDAR)
    //     transform.transform.rotation.x = 0.0;
    //     transform.transform.rotation.y = 0.0;
    //     transform.transform.rotation.z = 0.0;
    //     transform.transform.rotation.w = 1.0;

    //     tf_broadcaster_->sendTransform(transform);

    //     // Publish base_link → map transform for RViz visualization
    //     // geometry_msgs::msg::TransformStamped tf_map_base_link;
    //     // tf_map_base_link.header.stamp = this->get_clock()->now();
    //     // tf_map_base_link.header.frame_id = "map";
    //     // tf_map_base_link.child_frame_id = "base_link";

    //     // // Example quadcopter position (modify dynamically for actual movement)
    //     // tf_map_base_link.transform.translation.x = 0.0;
    //     // tf_map_base_link.transform.translation.y = 0.0;
    //     // tf_map_base_link.transform.translation.z = 1.0;  // Assuming 1m above ground

    //     // // Identity rotation (assuming level flight)
    //     // tf_map_base_link.transform.rotation.x = 0.0;
    //     // tf_map_base_link.transform.rotation.y = 0.0;
    //     // tf_map_base_link.transform.rotation.z = 0.0;
    //     // tf_map_base_link.transform.rotation.w = 1.0;

    //     // tf_broadcaster_->sendTransform(tf_map_base_link);
    // }

    // rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_px4_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_subscription_;
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

    // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4ScanTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
