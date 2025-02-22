#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import radians
import curses
import time

class TurtlebotGridController(Node):
    def __init__(self):
        super().__init__('turtlebot_grid_controller')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.15  # Reduced for better control
        self.angular_speed = 0.5  # radians per second
        self.grid_size = 1.0     # 1 meter per grid cell
        
        # Store current state
        self.is_moving = False
        self.current_velocity = Twist()
    
    def publish_velocity(self, vel_msg):
        """Publish velocity message and store current state"""
        self.current_velocity = vel_msg
        self.velocity_publisher.publish(vel_msg)
    
    def stop_robot(self):
        """Emergency stop function"""
        stop_msg = Twist()
        self.publish_velocity(stop_msg)
        self.is_moving = False
        time.sleep(0.1)  # Small delay to ensure stop command is received

    def move_straight(self, distance, is_forward=True):
        """Improved grid movement function"""
        if self.is_moving:
            self.stop_robot()
            
        vel_msg = Twist()
        direction = 1.0 if is_forward else -1.0
        vel_msg.linear.x = direction * self.linear_speed

        # Calculate time needed for movement (adding 10% to account for acceleration)
        duration = (distance / self.linear_speed) * 1.1
        
        # Execute movement
        self.is_moving = True
        start_time = time.time()
        
        while time.time() - start_time < duration and self.is_moving:
            self.publish_velocity(vel_msg)
            time.sleep(0.01)  # Reduced sleep time for more responsive control
            
        self.stop_robot()

    def rotate(self, angle_degrees, clockwise=True):
        """Improved rotation function"""
        if self.is_moving:
            self.stop_robot()
            
        vel_msg = Twist()
        direction = -1.0 if clockwise else 1.0
        vel_msg.angular.z = direction * self.angular_speed

        # Calculate rotation time (adding 10% to account for acceleration)
        duration = (radians(angle_degrees) / self.angular_speed) * 1.1
        
        # Execute rotation
        self.is_moving = True
        start_time = time.time()
        
        while time.time() - start_time < duration and self.is_moving:
            self.publish_velocity(vel_msg)
            time.sleep(0.01)
            
        self.stop_robot()

    def manual_control(self, key):
        """Improved manual control with immediate response"""
        vel_msg = Twist()
        
        if key == curses.KEY_UP:  # Forward
            vel_msg.linear.x = self.linear_speed
        elif key == curses.KEY_DOWN:  # Backward
            vel_msg.linear.x = -self.linear_speed
        elif key == curses.KEY_LEFT:  # Left
            vel_msg.angular.z = self.angular_speed
        elif key == curses.KEY_RIGHT:  # Right
            vel_msg.angular.z = -self.angular_speed
        elif key == ord(' '):  # Space bar to stop
            self.stop_robot()
            return
            
        self.publish_velocity(vel_msg)

def main():
    rclpy.init()
    robot = TurtlebotGridController()
    screen = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen.keypad(True)
    screen.timeout(100)  # Set timeout for getch() to make it non-blocking

    try:
        while rclpy.ok():
            screen.clear()
            screen.addstr("TurtleBot Grid Controller\n\n")
            screen.addstr("Select Mode:\n")
            screen.addstr("1 - Manual Teleoperation\n")
            screen.addstr("2 - Grid-based Navigation\n")
            screen.addstr("Press 'q' to quit.\n")
            screen.refresh()
            
            char = screen.getch()
            if char == -1:  # No key pressed
                continue
                
            if char == ord('q'):
                break
                
            elif char == ord('1'):
                # Manual Mode
                screen.clear()
                screen.addstr("Manual Mode:\n")
                screen.addstr("Use arrow keys to move\n")
                screen.addstr("SPACE - Stop robot\n")
                screen.addstr("x - Return to menu\n")
                screen.refresh()
                
                while True:
                    char = screen.getch()
                    if char == ord('x'):
                        robot.stop_robot()
                        break
                    elif char != -1:  # Only process valid key presses
                        robot.manual_control(char)
                    rclpy.spin_once(robot, timeout_sec=0)
                    
            elif char == ord('2'):
                # Grid Mode with improved responsiveness
                screen.clear()
                screen.addstr("Grid Navigation Mode\n\n")
                screen.addstr("Commands:\n")
                screen.addstr("f - Forward one grid (1m)\n")
                screen.addstr("b - Back one grid (1m)\n")
                screen.addstr("l - Turn left 90 degrees\n")
                screen.addstr("r - Turn right 90 degrees\n")
                screen.addstr("SPACE - Emergency Stop\n")
                screen.addstr("x - Return to menu\n\n")
                screen.refresh()
                
                while True:
                    char = screen.getch()
                    if char == -1:
                        rclpy.spin_once(robot, timeout_sec=0)
                        continue
                        
                    if char == ord('x'):
                        robot.stop_robot()
                        break
                    elif char == ord(' '):
                        robot.stop_robot()
                    elif char == ord('f'):
                        robot.move_straight(robot.grid_size)
                    elif char == ord('b'):
                        robot.move_straight(robot.grid_size, False)
                    elif char == ord('l'):
                        robot.rotate(90, False)
                    elif char == ord('r'):
                        robot.rotate(90, True)
                        
                    rclpy.spin_once(robot, timeout_sec=0)

    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Clean up
        robot.stop_robot()
        curses.nocbreak()
        screen.keypad(False)
        curses.echo()
        curses.endwin()
        robot.destroy_node()
        rclpy.shutdown()

def entry_point():
    return main()

if __name__ == '__main__':
    main()
