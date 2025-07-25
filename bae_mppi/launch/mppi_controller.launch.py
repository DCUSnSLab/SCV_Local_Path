"""
Launch file for MPPI controller
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directory
    pkg_dir = FindPackageShare('bae_mppi')
    
    # Default config file
    default_config_file = PathJoinSubstitution([
        pkg_dir, 'config', 'mppi_params.yaml'
    ])
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to MPPI controller config file'
    )
    
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='false',
        description='Whether to use GPU acceleration'
    )
    
    # MPPI controller node
    mppi_controller_node = Node(
        package='bae_mppi',
        executable='mppi_controller_node.py',
        name='mppi_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        use_gpu_arg,
        mppi_controller_node,
    ])