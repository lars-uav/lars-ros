import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tanayrs/lars-ros/lars_ws2/src/turtlebot3_grid_nav/install/turtlebot3_grid_nav'
