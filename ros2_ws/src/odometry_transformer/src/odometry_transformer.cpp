#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include "rclcpp/qos.hpp"

using namespace px4_msgs::msg;

class OdometryTransformer : public rclcpp::Node {
public:
    OdometryTransformer() : Node("odometry_transformer") {

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        // rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).BestEffort();  // Change QoS as needed

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
        subscription_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile,
            std::bind(&OdometryTransformer::odometryCallback, this, std::placeholders::_1)
        );

        // this->declare_parameter("use_sim_time", true);  // Enable simulation time
    }

private:
    // void odometryCallback(const VehicleOdometry::SharedPtr msg) {

    //     auto odom_msg = nav_msgs::msg::Odometry();
    //     odom_msg.header.stamp = this->get_clock()->now();
    //     odom_msg.header.frame_id = "odom";
    //     odom_msg.child_frame_id = "base_footprint";

    //     // Copy position data
    //     odom_msg.pose.pose.position.x = msg->position[0];
    //     odom_msg.pose.pose.position.y = msg->position[1];
    //     odom_msg.pose.pose.position.z = msg->position[2];

    //     // Copy orientation (assuming quaternion format)
    //     odom_msg.pose.pose.orientation.x = msg->q[0];
    //     odom_msg.pose.pose.orientation.y = msg->q[1];
    //     odom_msg.pose.pose.orientation.z = msg->q[2];
    //     odom_msg.pose.pose.orientation.w = msg->q[3];

    //     // Copy velocity
    //     odom_msg.twist.twist.linear.x = msg->velocity[0];
    //     odom_msg.twist.twist.linear.y = msg->velocity[1];
    //     odom_msg.twist.twist.linear.z = msg->velocity[2];

    //     publisher_->publish(odom_msg);
    //     RCLCPP_INFO(this->get_logger(), "Republishing vehicle odometry data to odom (x=%.2f, y=%.2f, z=%.2f)", msg->position[0], msg->position[1], msg->position[2]);
    // }

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

        // Publish transformed odometry
        publisher_->publish(publish_data);

        RCLCPP_INFO(this->get_logger(), "Published ENU odometry data (x=%.2f, y=%.2f, z=%.2f)",
                    publish_data.pose.pose.position.x, publish_data.pose.pose.position.y, publish_data.pose.pose.position.z);
    }


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}