
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace px4_msgs::msg;

class PX4ToPoseConverter : public rclcpp::Node {
public:
    PX4ToPoseConverter() : Node("px4_to_pose_converter") {

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        // Subscriber to PX4 VehicleLocalPosition
        subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            std::bind(&PX4ToPoseConverter::callback_vehicle_local_position, this, std::placeholders::_1));

        // Publisher for PoseStamped
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vehicle/pose", 10);
    }

private:
    void callback_vehicle_local_position(const VehicleLocalPosition::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map"; // Change based on your reference frame

        pose_msg.pose.position.x = msg->x;
        pose_msg.pose.position.y = msg->y;
        pose_msg.pose.position.z = msg->z;

        // PX4 does not provide orientation, so set it to identity (no rotation)
        pose_msg.pose.orientation.w = 1.0;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;

        publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published PoseStamped: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
    }

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4ToPoseConverter>());
    rclcpp::shutdown();
    return 0;
}