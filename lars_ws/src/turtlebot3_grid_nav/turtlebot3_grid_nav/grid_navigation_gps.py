import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from math import radians, sqrt, atan2, sin, cos, pi
import curses
import time

class TurtlebotGridController(Node):
    def __init__(self):
        super().__init__('turtlebot_grid_controller')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.grid_size = 1.0
        
        self.current_gps = None
        self.is_moving = False
    
    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
    
    def publish_velocity(self, vel_msg):
        self.velocity_publisher.publish(vel_msg)
    
    def stop_robot(self):
        stop_msg = Twist()
        self.publish_velocity(stop_msg)
        self.is_moving = False
        time.sleep(0.1)
    
    def move_straight(self, distance):
        vel_msg = Twist()
        vel_msg.linear.x = self.linear_speed
        duration = (distance / self.linear_speed) * 1.1
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publish_velocity(vel_msg)
            time.sleep(0.01)
        self.stop_robot()
    
    def rotate(self, angle_degrees):
        vel_msg = Twist()
        direction = -1.0 if angle_degrees > 0 else 1.0
        vel_msg.angular.z = direction * self.angular_speed
        duration = (radians(abs(angle_degrees)) / self.angular_speed) * 1.1
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publish_velocity(vel_msg)
            time.sleep(0.01)
        self.stop_robot()
    
    def gps_navigate(self, goal_lat, goal_lon):
        if self.current_gps is None:
            print("Waiting for GPS signal...")
            return
        
        lat1, lon1 = self.current_gps
        lat2, lon2 = goal_lat, goal_lon
        
        # Convert degrees to radians
        lat1, lon1, lat2, lon2 = map(lambda x: x * pi / 180.0, [lat1, lon1, lat2, lon2])
        
        # Haversine formula to get distance in meters
        R = 6371000  # Earth radius in meters
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = R * c
        
        # Compute heading angle (bearing)
        y = sin(lon2 - lon1) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1)
        heading = atan2(y, x) * (180 / pi)
        
        print(f"Moving towards {goal_lat}, {goal_lon} (Distance: {distance:.2f}m, Heading: {heading:.2f} degrees)")
        
        # Rotate to face target direction
        self.rotate(heading)
        self.move_straight(distance)


def main():
    rclpy.init()
    robot = TurtlebotGridController()
    screen = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen.keypad(True)
    
    try:
        while rclpy.ok():
            screen.clear()
            screen.addstr("TurtleBot Grid Controller\n\n")
            screen.addstr("1 - Manual Teleoperation\n")
            screen.addstr("2 - Grid-based Navigation\n")
            screen.addstr("3 - GPS Navigation\n")
            screen.addstr("Press 'q' to quit.\n")
            screen.refresh()
            
            char = screen.getch()
            if char == ord('q'):
                break
            elif char == ord('3'):
                screen.clear()
                screen.addstr("GPS Navigation Mode\n")
                screen.addstr("Enter Goal Latitude: ")
                curses.echo()
                goal_lat = float(screen.getstr().decode('utf-8'))
                screen.addstr("Enter Goal Longitude: ")
                goal_lon = float(screen.getstr().decode('utf-8'))
                curses.noecho()
                robot.gps_navigate(goal_lat, goal_lon)
            
            rclpy.spin_once(robot, timeout_sec=0)
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        robot.stop_robot()
        curses.nocbreak()
        screen.keypad(False)
        curses.echo()
        curses.endwin()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
