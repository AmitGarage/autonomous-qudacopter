import rclpy
from rclpy.node import Node
import subprocess

class MicroAgentNode(Node):
    def __init__(self):
        super().__init__('microagent')

        # Declare parameters
        self.declare_parameter('mode', 'serial')
        self.declare_parameter('serial_device', '/dev/serial0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('udp_port', 8888)

        # Get parameters
        mode = self.get_parameter('mode').get_parameter_value().string_value

        if mode == "serial":
            serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
            baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
            command = f"MicroXRCEAgent serial --dev {serial_device} -b {baud_rate}"
        elif mode == "udp":
            udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
            command = f"MicroXRCEAgent udp4 -p {udp_port} -v"
        else:
            self.get_logger().error("Invalid mode specified!")
            return

        self.get_logger().info(f"Running: {command}")

        try:
            # Run the command
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to start MicroXRCEAgent: {e}")

def main():
    rclpy.init()
    node = MicroAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
