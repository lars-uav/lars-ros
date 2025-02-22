import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Quaternion
from math import radians


class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('turtlebot_mover')
        
        # Initialize the navigator for navigation
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def move_to_grid(self, grid_num, speed=0.2):
        # Define grid positions, you can change these based on your setup
        grid_positions = {
            1: (1.0, 1.0),   # Grid 1 -> (1, 1)
            2: (2.0, 2.0),   # Grid 2 -> (2, 2)
            3: (3.0, 3.0),   # Grid 3 -> (3, 3)
            # Add other grid points here...
        }

        if grid_num not in grid_positions:
            self.get_logger().error(f"Grid number {grid_num} not found.")
            return

        goal_x, goal_y = grid_positions[grid_num]

        # Create the goal pose with the target x, y coordinates
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"  # Assuming your map frame is "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation = Quaternion(w=1.0)  # Assuming no rotation, facing forward

        # Send the goal to the navigator
        self.navigator.go_to_pose(goal_pose)

        # Wait for the navigation task to finish
        while not self.navigator.is_task_complete():
            feedback = self.navigator.get_feedback()
            if feedback.status == 'SUCCEEDED':
                self.get_logger().info("Reached target grid!")
                break
            elif feedback.status == 'CANCELED':
                self.get_logger().warn("Navigation was canceled.")
                break
            rclpy.spin_once(self)

    def _calculate_distance(self, current_position, goal_position):
        dx = current_position.x - goal_position.position.x
        dy = current_position.y - goal_position.position.y
        return (dx**2 + dy**2)**0.5


def main(args=None):
    rclpy.init(args=args)

    turtlebot_mover = TurtleBotMover()

    # Move to grid 2 at speed 0.2 m/s
    turtlebot_mover.move_to_grid(2, speed=0.2)

    rclpy.spin(turtlebot_mover)

    turtlebot_mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

