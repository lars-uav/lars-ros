#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps_data',  # This is the topic where GPS data is being published
            self.gps_callback,
            10)
        self.subscription

    def gps_callback(self, msg):
        self.get_logger().info(f"Received GPS Coordinates: Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}")

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

