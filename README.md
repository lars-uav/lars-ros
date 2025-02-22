# LARS TurtleBot3 Repository

This repository contains a modified version of the TurtleBot3 ROS2 packages customized for the LARS GroundBot. It includes the standard TurtleBot3 packages with specific modifications to accommodate the LARS GroundBot design, plus a custom navigation package.

## Repository Structure (under lars_ws)

```
.
├── DynamixelSDK/          # Dynamixel SDK for motor control
├── turtlebot3/           # Core TurtleBot3 packages with LARS modifications
├── turtlebot3_grid_nav/  # Custom grid-based navigation package
├── turtlebot3_msgs/      # TurtleBot3 message definitions
└── turtlebot3_simulations/ # Simulation packages with LARS modifications
```

## Modifications

### Physical Model Changes

The following modifications have been made to adapt the TurtleBot3 model for the LARS GroundBot:

1. In `turtlebot3/turtlebot3_description/`:
   - Replaced `meshes/waffle_base.stl` with LARS GroundBot STL
   - Elevated LiDAR position by 0.04m in Z direction
   - Removed visual meshes for left and right wheels

2. In `turtlebot3_simulations/turtlebot3_gazebo/`:
   - Updated `models/turtlebot3_common` with matching changes
   - Updated `models/turtlebot3_waffle` with matching changes

### Custom Packages

#### turtlebot3_grid_nav

A custom package that implements grid-based navigation for the robot. Features include:

- Manual teleoperation mode
- Grid-based movement (1m x 1m grid)
- Keyboard control interface
- Emergency stop functionality

##### Usage

1. **Manual Control Mode**:
   - Arrow keys for movement
   - Space bar for emergency stop
   - 'x' to return to main menu

2. **Grid Navigation Mode**:
   - 'f' - Move forward one grid (1m)
   - 'b' - Move backward one grid (1m)
   - 'l' - Turn left 90 degrees
   - 'r' - Turn right 90 degrees
   - Space bar for emergency stop
   - 'x' to return to main menu

## Dependencies

- ROS2
- Python 3
- curses library
- TurtleBot3 dependencies

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/your_ws/src
git clone <repository_url>
```

2. Build the workspace:
```bash
cd ~/your_ws
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Running the Grid Navigation

To start the grid navigation system:
```bash
ros2 run turtlebot3_grid_nav grid_navigation
```

## Original Packages

This repository is based on the following original TurtleBot3 packages:
- [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [TurtleBot3 Messages](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

## Contributors

- Original TurtleBot3 packages by ROBOTIS
- LARS GroundBot modifications and grid navigation package by TanayRS
