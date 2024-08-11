#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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
import numpy as np

class SenderNode(Node):
    def __init__(self):
        super().__init__('sender_node')
        self.bridge = CvBridge()
        
        # Declare and get parameters
        self.declare_parameter('yolo_model', '/home/yan/Downloads/best.pt')
        self.declare_parameter('server_ip', '0.0.0.0')  # Listen on all interfaces
        self.declare_parameter('server_port', 5000)
        self.declare_parameter('target_fps', 10)
        self.declare_parameter('max_queue_size', 30)
        self.declare_parameter('optimal_queue_size', 10)
        self.declare_parameter('drop_threshold', 25)
        self.declare_parameter('yolo_input_size', [640, 640])
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('specified_classes', ['Orange'])  

        self.model = YOLO(self.get_parameter('yolo_model').value)
        self.server_ip = self.get_parameter('server_ip').value
        self.server_port = self.get_parameter('server_port').value
        self.target_fps = self.get_parameter('target_fps').value
        self.max_queue_size = self.get_parameter('max_queue_size').value
        self.optimal_queue_size = self.get_parameter('optimal_queue_size').value
        self.drop_threshold = self.get_parameter('drop_threshold').value
        self.yolo_input_size = tuple(self.get_parameter('yolo_input_size').value)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.specified_classes = self.get_parameter('specified_classes').value

        # Socket setup
        self.server_socket = self.start_server(self.server_ip, self.server_port)
        self.client_socket = None

        # Frame and result queues
        self.frame_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # ROS Publishers
        self.detection_pub = self.create_publisher(String, 'yolo_detections', 10)
        self.received_image_pub = self.create_publisher(Image, 'received_image', 10)

        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_log_time = time.time()

        # Frame rate control
        self.frame_interval = 1.0 / self.target_fps

        # Start worker threads
        threading.Thread(target=self.process_frames, daemon=True).start()
        threading.Thread(target=self.send_results, daemon=True).start()

        self.get_logger().info('Sender Node initialized and ready to process frames')

    def start_server(self, ip, port):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((ip, port))
        server_socket.listen(1)
        self.get_logger().info(f"Server started on {ip}:{port}")
        return server_socket

    def run(self):
        while rclpy.ok():
            self.get_logger().info("Waiting for client connection...")
            self.client_socket, addr = self.server_socket.accept()
            self.get_logger().info(f"Connected to client: {addr}")
            try:
                self.handle_client()
            except Exception as e:
                self.get_logger().error(f"Error handling client: {str(e)}")
            finally:
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None

    def handle_client(self):
        try:
            while rclpy.ok():
                frame = self.receive_frame()
                if frame is None:
                    self.get_logger().info("Client disconnected")
                    break
                
                # Adaptive queue management
                current_queue_size = self.frame_queue.qsize()
                if current_queue_size < self.max_queue_size:
                    self.frame_queue.put(frame)
                    if current_queue_size > self.drop_threshold:
                        try:
                            self.frame_queue.get_nowait()
                            self.get_logger().warn("Dropped oldest frame due to high queue size")
                        except queue.Empty:
                            pass
                else:
                    self.get_logger().warn("Max queue size reached, dropping incoming frame")
                
                # Publish received image to ROS topic
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.received_image_pub.publish(ros_image)
                
                # Log queue size periodically
                current_time = time.time()
                if current_time - self.last_log_time > 5:
                    self.get_logger().info(f"Current frame queue size: {current_queue_size}")
                    self.last_log_time = current_time
        
        except Exception as e:
            self.get_logger().error(f"Error handling client: {str(e)}")

    def receive_frame(self):
        try:
            msg_size = struct.unpack("L", self.client_socket.recv(struct.calcsize("L")))[0]
            data = b""
            while len(data) < msg_size:
                packet = self.client_socket.recv(min(msg_size - len(data), 4096))
                if not packet:
                    return None
                data += packet
            frame = pickle.loads(data)
            return frame
        except (struct.error, pickle.UnpicklingError, EOFError):
            return None

    def process_frames(self):
        last_process_time = time.time()
        while rclpy.ok():
            current_time = time.time()
            if current_time - last_process_time >= self.frame_interval:
                frames_processed = 0
                while not self.frame_queue.empty() and frames_processed < self.target_fps:
                    frame = self.frame_queue.get()
                    resized_frame = cv2.resize(frame, self.yolo_input_size)
                    result = self.process_with_yolo(resized_frame)
                    self.result_queue.put(result)
                    self.print_detections(result)
                    frames_processed += 1
                    self.frame_count += 1

                if frames_processed > 0:
                    fps = frames_processed / (current_time - last_process_time)
                    self.get_logger().info(f"Processed {frames_processed} frames. FPS: {fps:.2f}")

                # Adaptive processing rate
                current_queue_size = self.frame_queue.qsize()
                if current_queue_size > self.optimal_queue_size:
                    self.target_fps = min(30, self.target_fps + 1)
                elif current_queue_size < self.optimal_queue_size and self.target_fps > 1:
                    self.target_fps = max(1, self.target_fps - 1)
                self.frame_interval = 1.0 / self.target_fps

                last_process_time = current_time
            else:
                time.sleep(0.001)

    def process_with_yolo(self, frame):
        results = self.model(frame)
        return results

    def print_detections(self, results):
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                self.get_logger().info(f"Detected: {self.model.names[cls]} with confidence {conf:.2f}")

    def send_results(self):
        while rclpy.ok():
            if not self.result_queue.empty() and self.client_socket:
                result = self.result_queue.get()
                detections = []
                for r in result:
                    boxes = r.boxes
                    for box in boxes:
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        detections.append({
                            'class': self.model.names[cls],
                            'confidence': conf,
                            'bbox': [x1, y1, x2, y2]
                        })
                
                # Send detections to client over socket
                try:
                    detection_data = json.dumps(detections).encode('utf-8')
                    msg_size = struct.pack("L", len(detection_data))
                    self.client_socket.sendall(msg_size + detection_data)
                    self.get_logger().info(f"Sent detection results to client")
                except Exception as e:
                    self.get_logger().error(f"Error sending detection results: {str(e)}")

                # Publish detections to ROS topic (keep this for local use)
                detection_msg = String()
                detection_msg.data = json.dumps(detections)
                self.detection_pub.publish(detection_msg)
            else:
                time.sleep(0.01)

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