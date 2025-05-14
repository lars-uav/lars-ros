#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import zmq
import pickle
import numpy as np
import time

class TopicClient(Node):
    def __init__(self, robot_ip):
        super().__init__('topic_client')
        
        # ZMQ setup
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{robot_ip}:5555")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"scan")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"odom")
        
        # Create publishers
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10)
            
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10)
        
        # Create a timer to check for messages
        self.timer = self.create_timer(0.001, self.check_messages)
        
        self.get_logger().info(f'Topic client connected to {robot_ip}:5555')
    
    def check_messages(self):
        try:
            # Non-blocking receive
            topic, data = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            msg_data = pickle.loads(data)
            
            if topic == b"scan":
                self.publish_scan(msg_data)
            elif topic == b"odom":
                self.publish_odom(msg_data)
        except zmq.Again:
            # No message available
            pass
    
    def publish_scan(self, data):
        msg = LaserScan()
        
        # Fill header
        msg.header.stamp.sec = data['header']['stamp']['sec']
        msg.header.stamp.nanosec = data['header']['stamp']['nanosec']
        msg.header.frame_id = data['header']['frame_id']
        
        # Fill scan data
        msg.angle_min = data['angle_min']
        msg.angle_max = data['angle_max']
        msg.angle_increment = data['angle_increment']
        msg.time_increment = data['time_increment']
        msg.scan_time = data['scan_time']
        msg.range_min = data['range_min']
        msg.range_max = data['range_max']
        msg.ranges = data['ranges']
        msg.intensities = data['intensities']
        
        self.scan_pub.publish(msg)
        self.get_logger().debug('Published scan message')
    
    def publish_odom(self, data):
        msg = Odometry()
        
        # Fill header
        msg.header.stamp.sec = data['header']['stamp']['sec']
        msg.header.stamp.nanosec = data['header']['stamp']['nanosec']
        msg.header.frame_id = data['header']['frame_id']
        
        # Fill child frame id
        msg.child_frame_id = data['child_frame_id']
        
        # Fill pose
        msg.pose.pose.position.x = data['pose']['pose']['position']['x']
        msg.pose.pose.position.y = data['pose']['pose']['position']['y']
        msg.pose.pose.position.z = data['pose']['pose']['position']['z']
        
        msg.pose.pose.orientation.x = data['pose']['pose']['orientation']['x']
        msg.pose.pose.orientation.y = data['pose']['pose']['orientation']['y']
        msg.pose.pose.orientation.z = data['pose']['pose']['orientation']['z']
        msg.pose.pose.orientation.w = data['pose']['pose']['orientation']['w']
        
        # Fill twist
        msg.twist.twist.linear.x = data['twist']['twist']['linear']['x']
        msg.twist.twist.linear.y = data['twist']['twist']['linear']['y']
        msg.twist.twist.linear.z = data['twist']['twist']['linear']['z']
        
        msg.twist.twist.angular.x = data['twist']['twist']['angular']['x']
        msg.twist.twist.angular.y = data['twist']['twist']['angular']['y']
        msg.twist.twist.angular.z = data['twist']['twist']['angular']['z']
        
        self.odom_pub.publish(msg)
        self.get_logger().debug('Published odom message')

def main(args=None):
    rclpy.init(args=args)
    
    # Get robot IP from command line or use default
    import sys
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.100"  # Replace with your robot's IP
    
    node = TopicClient(robot_ip)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

