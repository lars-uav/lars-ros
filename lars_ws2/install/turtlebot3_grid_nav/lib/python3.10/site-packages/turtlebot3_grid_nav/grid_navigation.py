import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class GridNavigator(Node):
    def __init__(self):
        super().__init__('grid_navigator')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.moving = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if self.moving:
            if abs(self.current_x - self.target_x) < 0.05 and abs(self.current_y - self.target_y) < 0.05:
                self.stop()
                self.moving = False

    def move_to_grid(self, grid_x, grid_y):
        self.target_x = grid_x
        self.target_y = grid_y
        self.moving = True
        twist = Twist()
        
        if self.current_x < self.target_x:
            twist.linear.x = 0.2
        elif self.current_x > self.target_x:
            twist.linear.x = -0.2
        elif self.current_y < self.target_y:
            twist.linear.y = 0.2
        elif self.current_y > self.target_y:
            twist.linear.y = -0.2
        
        self.cmd_vel_pub.publish(twist)
    
    def stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Reached target grid cell")

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Enter grid coordinates (x,y): ")
                grid_x, grid_y = map(int, user_input.split(','))
                self.move_to_grid(grid_x, grid_y)
            except ValueError:
                self.get_logger().error("Invalid input format. Use x,y")

def main():
    rclpy.init()
    navigator = GridNavigator()
    navigator.input_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
