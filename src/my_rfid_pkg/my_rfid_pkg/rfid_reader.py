import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import sys

class RFIDReader(Node):
    def __init__(self):
        super().__init__('rfid_reader')
        self.publisher_ = self.create_publisher(String, 'rfid_data', 10)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM2', 9600, timeout=2)
        except serial.SerialException:
            self.get_logger().error("Serial port not available")
            sys.exit(1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:  # ensure that line is not empty
                self.publisher_.publish(String(data=line))
                self.get_logger().info(f"Published: {line}")

def main(args=None):
    rclpy.init(args=args)
    rfid_reader = RFIDReader()
    rclpy.spin(rfid_reader)
    rfid_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

