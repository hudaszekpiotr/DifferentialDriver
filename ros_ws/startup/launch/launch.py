from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu',
            executable='main',
            name='imu_node'
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ldlidar_stl_ros2'), 'launch'),
         '/ld19.launch.py'])),
         
        Node(
            package='motors_driver',
            executable='main',
            name='motors_driver_node',
        ),
        Node(
            package='odometry',
            executable='main',
            name='odometry_node',
        ),
        Node(
            package='twist_to_wheels_speed',
            executable='main',
            name='twist_to_wheels_speed_node',
        ),
    ])
