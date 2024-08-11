#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import json
import socket
import struct
import pickle
import threading
import queue

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera_image', 10)
        self.detection_pub = self.create_publisher(String, 'yolo_detections', 10)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Socket setup
        self.server_ip = '192.168.137.143'  # Sender IP
        self.server_port = 5000
        self.client_socket = None
        self.connect_to_server()

        # Frame and result queues
        self.frame_queue = queue.Queue(maxsize=10)
        self.result_queue = queue.Queue()

        # Start worker threads
        threading.Thread(target=self.send_frames, daemon=True).start()
        threading.Thread(target=self.receive_results, daemon=True).start()

        # Create a timer to run the main loop
        self.create_timer(1/30, self.timer_callback)  # 30 Hz

    def connect_to_server(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while rclpy.ok():
            try:
                self.client_socket.connect((self.server_ip, self.server_port))
                self.get_logger().info(f"Connected to server at {self.server_ip}:{self.server_port}")
                break
            except socket.error:
                self.get_logger().warn(f"Couldn't connect to server at {self.server_ip}:{self.server_port}. Retrying in 5 seconds...")
                rclpy.spin_once(self, timeout_sec=5.0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame. Retrying...")
            return

        # Publish to ROS topic
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(ros_image)
        
        # Add frame to queue for sending
        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            self.get_logger().warn("Frame queue full, skipping frame")

        # Process latest result if available
        try:
            result = self.result_queue.get_nowait()
            self.process_result(result)
        except queue.Empty:
            pass

    def send_frames(self):
        while rclpy.ok():
            frame = self.frame_queue.get()
            data = pickle.dumps(frame)
            message_size = struct.pack("L", len(data))
            try:
                self.client_socket.sendall(message_size + data)
            except socket.error:
                self.get_logger().warn("Lost connection to server. Attempting to reconnect...")
                self.connect_to_server()

    def receive_results(self):
        while rclpy.ok():
            try:
                data = b""
                payload_size = struct.calcsize("L")
                while len(data) < payload_size:
                    packet = self.client_socket.recv(4096)
                    if not packet:
                        raise socket.error("Connection closed by server")
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("L", packed_msg_size)[0]
                while len(data) < msg_size:
                    data += self.client_socket.recv(4096)
                frame_data = data[:msg_size]
                result = json.loads(frame_data.decode('utf-8'))
                self.result_queue.put(result)
            except socket.error as e:
                self.get_logger().warn(f"Error receiving result: {e}. Reconnecting...")
                self.connect_to_server()

    def process_result(self, result):
        self.get_logger().info(f"Received YOLO result: {result}")
        
        # Publish YOLO detections as a JSON string
        detection_msg = String()
        detection_msg.data = json.dumps(result)
        self.detection_pub.publish(detection_msg)

    def __del__(self):
        self.cap.release()
        if self.client_socket:
            self.client_socket.close()

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    try:
        rclpy.spin(client_node)
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()