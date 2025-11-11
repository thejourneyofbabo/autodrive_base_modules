#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('autodrive_ackermann')
    config_file = os.path.join(pkg_share, 'config', 'autodrive_params.yaml')

    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='roboracer_1',
        description='Name of the vehicle in autodrive simulator'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )

    # Get launch configurations
    vehicle_name = LaunchConfiguration('vehicle_name')
    config_file_path = LaunchConfiguration('config_file')

    # Ackermann to Autodrive converter node
    ackermann_to_autodrive_node = Node(
        package='autodrive_ackermann',
        executable='ackermann_to_autodrive_node',
        name='ackermann_to_autodrive',
        output='screen',
        parameters=[
            config_file_path,
            {'vehicle_name': vehicle_name}
        ]
    )

    # Autodrive to Odometry node
    autodrive_to_odom_node = Node(
        package='autodrive_ackermann',
        executable='autodrive_to_odom_node',
        name='autodrive_to_odom',
        output='screen',
        parameters=[
            config_file_path,
            {'vehicle_name': vehicle_name}
        ]
    )

    return LaunchDescription([
        vehicle_name_arg,
        config_file_arg,
        ackermann_to_autodrive_node,
        autodrive_to_odom_node,
    ])
