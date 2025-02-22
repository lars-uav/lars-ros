cmake_minimum_required(VERSION 3.8)
project(lars_groundbot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and Xacro files
install(
  DIRECTORY urdf meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Make launch files executable
install(
  PROGRAMS
    launch/spawn_lars_groundbot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
