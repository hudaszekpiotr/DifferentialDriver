import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    camera_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'camera.yaml'
    )

    camera_node = Node(
        package='v4l2_camera',
        name='v4l2_camera_node',
        executable='v4l2_camera_node',
        parameters=[camera_config],
    )

    camera_calibration_node = Node(
        package='camera_calibration',
        name='cameracalibrator',
        executable='cameracalibrator',
        arguments=[
               '--size', '10x8',
               '--square', '0.02'
               ],
        remappings=[
            ('/image', '/image_raw'),
            ('/camera', '/camera_info'),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(camera_node)
    ld.add_action(camera_calibration_node)

    return ld
