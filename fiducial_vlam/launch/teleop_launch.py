import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Full teleop launch: driver, base, joystick, etc.


def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(cmd=['rviz2', '-d', 'src/fiducial_vlam/fiducial_vlam/cfg/default.rviz'], output='screen'),
        ExecuteProcess(cmd=['rviz2'], output='screen'),
        Node(package='tello_driver', node_executable='tello_driver', output='screen'),
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen'),
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[{
            'use_sim_time': False,                           # Use /clock if available
            'publish_tfs': 1,                               # Publish marker /tf
            'marker_length': 0.1778,                        # Marker length
            'marker_map_load_full_filename': 'fiducial_marker_locations.yaml',      # Load a pre-built map from disk
            'make_not_use_map': 0                           # Don't save a map to disk
        }]),
        ])
