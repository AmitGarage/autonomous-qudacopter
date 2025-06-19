#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
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
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        // Subscriber to PX4 VehicleLocalPosition
        // subscription_px4_ = this->create_subscription<VehicleLocalPosition>(
        //     "/fmu/out/vehicle_local_position", qos_profile,
        //     std::bind(&PX4ScanTFBroadcaster::callback_vehicle_local_position, this, std::placeholders::_1));

        // this->declare_parameter("use_sim_time", true);  // Enable simulation time
        
        // Subscriber to LaserScan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile,
            std::bind(&PX4ScanTFBroadcaster::callback_scan, this, std::placeholders::_1));

        // Publisher for modified scan data (same topic as original)
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        odom_subscription_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile,
            std::bind(&PX4ScanTFBroadcaster::odometryCallback, this, std::placeholders::_1)
        );

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
        publish_data.header.frame_id = "odom";
        publish_data.child_frame_id = "base_footprint";

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
        publish_data.pose.pose.orientation.x = msg->q[0];
        publish_data.pose.pose.orientation.y = msg->q[1];
        publish_data.pose.pose.orientation.z = msg->q[2];
        publish_data.pose.pose.orientation.w = msg->q[3];
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
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";

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
        modified_scan.header.frame_id = "x500_lidar_2d_0/link/lidar_2d_v2";  // Mapping scan data to base_link

        // Publish modified scan data back on the same topic
        scan_publisher_->publish(modified_scan);

        RCLCPP_INFO(this->get_logger(), "Republished scan data with updated frame_id on /scan");

        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "base_footprint";
        transform.child_frame_id = "x500_lidar_2d_0/link/lidar_2d_v2";

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
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4ScanTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
