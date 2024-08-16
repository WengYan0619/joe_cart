#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import socket
import struct
import pickle
import json
import time
import threading
import numpy as np

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        
        # Declare and get parameters
        self.declare_parameter('server_ip', '10.42.0.1')
        self.declare_parameter('server_port', 5000)
        self.declare_parameter('target_fps', 15)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('camera_index', 3)
        self.declare_parameter('send_every_nth_frame', 3)
        self.declare_parameter('confidence_threshold', 0.75)

        self.server_ip = self.get_parameter('server_ip').value
        self.server_port = self.get_parameter('server_port').value
        self.target_fps = self.get_parameter('target_fps').value
        self.frame_size = (
            self.get_parameter('frame_width').value,
            self.get_parameter('frame_height').value
        )
        self.camera_index = self.get_parameter('camera_index').value
        self.send_every_nth_frame = self.get_parameter('send_every_nth_frame').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        self.socket = None
        self.camera = None
        self.connect_to_server()
        self.open_camera()

        # ROS Publisher for YOLO results
        self.detection_publisher = self.create_publisher(String, 'scene_change_detections', 10)
        
        self.receive_thread = threading.Thread(target=self.receive_results, daemon=True)
        self.receive_thread.start()

        self.send_thread = threading.Thread(target=self.send_frames, daemon=True)
        self.send_thread.start()

        # Initialize set to keep track of current scene detections
        self.current_scene_detections = set()
        
        self.get_logger().info('Client Node has been started')

    def connect_to_server(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.server_ip, self.server_port))
            self.get_logger().info(f"Connected to server at {self.server_ip}:{self.server_port}")
        except socket.error as e:
            self.get_logger().error(f"Failed to connect to server: {e}")
            self.socket = None

    def open_camera(self):
        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            self.get_logger().error("Failed to open camera")
            return
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_size[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_size[1])

    def send_frames(self):
        frame_count = 0
        while rclpy.ok():
            if self.camera is None or not self.camera.isOpened():
                self.get_logger().warn("Camera is not open")
                self.open_camera()
                time.sleep(1)
                continue

            ret, frame = self.camera.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame")
                time.sleep(0.1)
                continue

            frame_count += 1
            if frame_count % self.send_every_nth_frame == 0:
                if not self.send_frame(frame):
                    self.get_logger().warn("Failed to send frame, attempting to reconnect...")
                    self.connect_to_server()
            
            time.sleep(0.01)  # Small delay to prevent busy-waiting

    def send_frame(self, frame):
        if self.socket is None:
            return False
        resized_frame = cv2.resize(frame, self.frame_size)
        data = pickle.dumps(resized_frame)
        message_size = struct.pack("L", len(data))
        try:
            self.socket.sendall(message_size + data)
            return True
        except socket.error as e:
            self.get_logger().error(f"Error sending frame: {e}")
            return False

    def receive_results(self):
        while rclpy.ok():
            if self.socket is None:
                time.sleep(1)
                continue
            try:
                msg_size = struct.unpack("L", self.socket.recv(struct.calcsize("L")))[0]
                data = b""
                while len(data) < msg_size:
                    packet = self.socket.recv(min(msg_size - len(data), 4096))
                    if not packet:
                        break
                    data += packet
                detections = json.loads(data.decode('utf-8'))
                
                # Filter detections based on confidence threshold
                filtered_detections = [d for d in detections if d['confidence'] > self.confidence_threshold]
                
                # Create a set of detected classes
                new_scene_detections = set(d['class'] for d in filtered_detections)

                # Check if the scene has changed
                if new_scene_detections != self.current_scene_detections:
                    self.current_scene_detections = new_scene_detections
                    
                    # Publish only the first detection of each class
                    unique_detections = []
                    published_classes = set()
                    for detection in filtered_detections:
                        if detection['class'] not in published_classes:
                            unique_detections.append(detection)
                            published_classes.add(detection['class'])
                    
                    # ROS Publication
                    detection_msg = String()
                    detection_msg.data = json.dumps(unique_detections)
                    self.detection_publisher.publish(detection_msg)
                    self.get_logger().info(f"Published new scene detections: {unique_detections}")

            except Exception as e:
                self.get_logger().error(f"Error receiving detection results: {str(e)}")
                time.sleep(1)

    def cleanup(self):
        if self.camera is not None and self.camera.isOpened():
            self.camera.release()
        if self.socket is not None:
            self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    try:
        rclpy.spin(client_node)
    except KeyboardInterrupt:
        pass
    finally:
        client_node.cleanup()
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
