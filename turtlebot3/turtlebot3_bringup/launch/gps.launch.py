from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 9600,
                'frame_id': 'gps_link'
            }]
        )
    ])

