import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from coordinates_publisher.msg import traverse_coordinates
import time
import os

class CoordinatesPublisher(Node):
    def __init__(self):
        super().__init__('traverse_coordinates_publisher')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(traverse_coordinates, 'traverse_coordinates', qos_profile)
        self.filepath = "/home/amit-singh/Downloads/qudacopter/autonomous-qudacopter/ros2_ws/src/coordinates_publisher/config/traverse_coordinates.yaml"  # Replace with the actual file path
        self.last_modified = os.path.getmtime(self.filepath)
        self.timer = self.create_timer(1.0, self.check_file_change) # Check every 1 second

    def check_file_change(self):
        try:
            current_modified = os.path.getmtime(self.filepath)
            if current_modified != self.last_modified:
                self.last_modified = current_modified
                with open(self.filepath, 'r') as f:
                    file_content = f.read()
                msg = traverse_coordinates()
                msg.data = file_content
                self.publisher_.publish(msg)
                self.get_logger().info('Published : "%s"' % msg.data)
        except FileNotFoundError:
             self.get_logger().warn(f"File not found: {self.filepath}")

def main(args=None):
    rclpy.init(args=args)
    file_change_publisher = CoordinatesPublisher()
    rclpy.spin(file_change_publisher)
    file_change_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()