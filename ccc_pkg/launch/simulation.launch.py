import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_example = get_package_share_directory('ccc_pkg')
    robot_namespace = 'dolly_1'

    world_path = os.path.join(pkg_example, 'worlds', 'empty.world')
    dolly_model = os.path.join(pkg_example, 'models', 'dolly', 'model.sdf')

    return LaunchDescription([
        ExecuteProcess(cmd=[
            'gazebo',  # put gazebo if you want the gui / gzserver otherwise
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # publish /clock
            '-s', 'libgazebo_ros_factory.so',  # provide gazebo_ros::Node
            world_path
        ], output='screen'),


        # add robot model
        Node(package='ccc_pkg', executable='spawn_elements.py', output='screen',
             arguments=[dolly_model, '1', '5', '0', '0', robot_namespace]),


        # add robot controller
        #Node(package='ccc_pkg', executable='controller.py', output='screen', namespace=robot_namespace),
    ])
