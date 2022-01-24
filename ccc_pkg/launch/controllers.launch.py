import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_namespace = 'dolly_1'

    return LaunchDescription([
        # add robot controller
        Node(package='ccc_pkg', executable='controller.py',
             output='screen', namespace=robot_namespace),

    ])
