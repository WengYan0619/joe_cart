#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from ultralytics import YOLO
import json
import socket
import struct
import pickle
import threading
import queue
import time

class SenderNode(Node):
    def __init__(self):
        super().__init__('sender_node')
        self.bridge = CvBridge()
        self.model = YOLO('best.pt')  # Make sure 'best.pt' is in the same directory as this script
        
        # Socket setup
        self.server_socket = self.start_server(5000)
        self.client_socket = None

        # Frame and result queues
        self.frame_queue = queue.Queue()  # Removed maxsize limit
        self.result_queue = queue.Queue()

        # ROS Publishers
        self.detection_pub = self.create_publisher(String, 'yolo_detections', 10)
        self.received_image_pub = self.create_publisher(Image, 'received_image', 10)

        # Start worker threads
        threading.Thread(target=self.process_frames, daemon=True).start()
        threading.Thread(target=self.send_results, daemon=True).start()

        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_log_time = time.time()

    def start_server(self, port):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', port))
        server_socket.listen(1)
        self.get_logger().info(f"Server listening on port {port}")
        return server_socket

    def run(self):
        while rclpy.ok():
            self.get_logger().info("Waiting for a connection...")
            self.client_socket, addr = self.server_socket.accept()
            self.get_logger().info(f"Connected by {addr}")
            
            self.handle_client()

    def handle_client(self):
        try:
            while rclpy.ok():
                frame = self.receive_frame()
                if frame is None:
                    self.get_logger().info("Client disconnected")
                    break
                
                # Publish received image to ROS topic
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.received_image_pub.publish(ros_image)
                
                # Add frame to queue for processing
                self.frame_queue.put(frame)
                
                # Log queue size periodically
                current_time = time.time()
                if current_time - self.last_log_time > 5:  # Log every 5 seconds
                    self.get_logger().info(f"Current frame queue size: {self.frame_queue.qsize()}")
                    self.last_log_time = current_time
        
        except Exception as e:
            self.get_logger().error(f"Error handling client: {str(e)}")
        finally:
            if self.client_socket:
                self.client_socket.close()

    def receive_frame(self):
        data = b""
        payload_size = struct.calcsize("L")
        
        while len(data) < payload_size:
            packet = self.client_socket.recv(4096)
            if not packet:
                return None
            data += packet
        
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]
        
        while len(data) < msg_size:
            data += self.client_socket.recv(4096)
        
        frame_data = data[:msg_size]
        return pickle.loads(frame_data)

    def process_frames(self):
        while rclpy.ok():
            frame = self.frame_queue.get()
            result = self.process_with_yolo(frame)
            self.result_queue.put(result)
            self.print_detections(result)

            # Performance tracking
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.start_time >= 10:  # Log every 10 seconds
                fps = self.frame_count / (current_time - self.start_time)
                self.get_logger().info(f"FPS: {fps:.2f}")
                self.frame_count = 0
                self.start_time = current_time

    def process_with_yolo(self, frame):
        results = self.model(frame)
        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                class_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[class_id]
                detections.append({
                    "class": class_name,
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2]
                })
        return detections

    def send_results(self):
        while rclpy.ok():
            result = self.result_queue.get()
            self.send_result(result)
            self.publish_result(result)

    def send_result(self, result):
        result_json = json.dumps({"result": result})
        result_size = struct.pack("L", len(result_json))
        try:
            self.client_socket.sendall(result_size + result_json.encode())
        except socket.error:
            self.get_logger().warn("Error sending result. Client might have disconnected.")

    def publish_result(self, result):
        detection_msg = String()
        detection_msg.data = json.dumps(result)
        self.detection_pub.publish(detection_msg)

    def print_detections(self, detections):
        self.get_logger().info("\n--- Detections ---")
        for i, detection in enumerate(detections, 1):
            self.get_logger().info(f"{i}. Class: {detection['class']}, Confidence: {detection['confidence']:.2f}")
        self.get_logger().info("------------------\n")

    def __del__(self):
        if self.server_socket:
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    sender_node = SenderNode()
    try:
        sender_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        sender_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()