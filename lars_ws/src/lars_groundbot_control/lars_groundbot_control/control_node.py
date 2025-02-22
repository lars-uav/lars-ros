import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = Twist()
        msg.linear.x = 0.5  # Forward speed
        msg.angular.z = 0.2  # Turning speed
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        self.get_logger().info(f"Pose: {msg.pose.pose.position}")
        self.get_logger().info(f"Velocity: {msg.twist.twist.linear}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

