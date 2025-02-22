import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('lars_groundbot_description')

    world_file = os.path.join(pkg_description, 'worlds', 'lars_groundbot.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'lars_groundbot.urdf')

    return LaunchDescription([
        # Launch Gazebo with ROS integration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Spawn the Robot
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'lars_groundbot',
                '-file', urdf_file
            ],
            output='screen'
        )
    ])

