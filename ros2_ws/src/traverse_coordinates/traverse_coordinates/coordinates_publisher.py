import rclpy
from rclpy.node import Node
from custom_msgs.msg import TraverseCoordinates
import json  # Use JSON for structured input files
import os
import time

class CoordinatesPublisher(Node):
    """ROS 2 Publisher Node that reads a 2D array from a file and publishes a flattened MatrixMsg."""

    def __init__(self):
        super().__init__('coordinates_publisher')

        # Declare parameters
        self.declare_parameter('traverse_coordinates_file_path', '/home/amit-singh/Downloads/qudacopter/autonomous-qudacopter/start_time_offboard_control.json')

        # Load file path parameter
        self.traverse_coordinates_file_path = self.get_parameter('traverse_coordinates_file_path').get_parameter_value().string_value

        # Create publisher
        self.publisher_ = self.create_publisher(TraverseCoordinates, '/traverse_coordinates_topic', 10)

        # Publish at a fixed rate
        self.timer = self.create_timer(0.1, self.check_for_file_update)
    
    def is_matrix_empty(self):
        """Checks if the matrix is empty (0×0)."""
        try:
            with open(self.traverse_coordinates_file_path, 'r') as f:
                matrix = json.load(f)
                return not matrix or len(matrix) == 0 or len(matrix[0]) == 0
        except Exception as e:
            self.get_logger().error(f"Failed to read file: {e}")
            return True
        
    def check_for_file_update(self):
        """Checks if the file contains matrix of size greater than 0x0; publishes if greater."""

        # Publish only if file modification time has changed
        if not self.is_matrix_empty():
            self.publish_message()
    
    def publish_message(self):
        """Reads the file and publishes updated matrix data."""
        try:
            with open(self.traverse_coordinates_file_path, 'r') as f:
                matrix = json.load(f)
                rows = len(matrix)
                cols = len(matrix[0]) if rows > 0 else 0
        except Exception as e:
            self.get_logger().error(f"Failed to read file: {e}")
            return

        # Flatten matrix for publishing
        data = [num for row in matrix for num in row]

        msg = TraverseCoordinates()
        msg.data = data
        msg.rows = rows
        msg.cols = cols
        msg.file_name = self.traverse_coordinates_file_path

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Matrix {msg.rows}x{msg.cols} - Data={msg.data}")

        # Reset file content to an empty matrix (0x0)
        # self.reset_file()

    def reset_file(self):
        """Overwrites the file with an empty matrix."""
        try:
            # Write an empty matrix (0x0)
            with open(self.traverse_coordinates_file_path, 'w') as f:
                json.dump([], f)

            self.get_logger().info("Matrix file reset to 0x0")
        except Exception as e:
            self.get_logger().error(f"Failed to reset file: {e}")

def main():
    rclpy.init()
    node = CoordinatesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()