import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('lars_groundbot_description')

    world_file = os.path.join(pkg_description, 'worlds', 'lars_groundbot.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'lars_groundbot.urdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
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

