#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from tf_transformations import quaternion_from_euler

class OdomTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_transform_broadcaster')
        
        # Create a subscriber for the /odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Create a transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info('Odom transform broadcaster started')
    
    def odom_callback(self, msg):
        # Create transform from odom to base_footprint
        t = TransformStamped()
        
        # Set header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # Set translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Also create base_footprint to base_link transform (if needed)
        t2 = TransformStamped()
        t2.header.stamp = msg.header.stamp
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        
        # Set translation (typically a small z offset)
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.010  # Small z offset, adjust as needed
        
        # Set rotation (identity quaternion)
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t2)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

