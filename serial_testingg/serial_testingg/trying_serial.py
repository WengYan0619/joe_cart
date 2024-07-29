import rclpy
from rclpy.node import Node
import serial

class SerialListenerNode(Node):
    def __init__(self):
        super().__init__('serial_listener_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check for new data every 0.1 seconds

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f'Received from Arduino: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

