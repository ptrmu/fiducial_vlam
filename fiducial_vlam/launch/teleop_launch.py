import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Full teleop launch: driver, base, joystick, etc.

tello_driver_args = [
    {
        'drone_ip': '192.168.0.21',
    }
]

fiducial_vlam_path = get_package_share_directory('fiducial_vlam')
map_path = os.path.join(fiducial_vlam_path, 'launch', 'fiducial_marker_locations_office.yaml')


def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(cmd=['rviz2', '-d', 'src/fiducial_vlam/fiducial_vlam/cfg/default.rviz'], output='screen'),
        ExecuteProcess(cmd=['rviz2'], output='screen'),
        Node(package='tello_driver', node_executable='tello_driver_main', output='screen',
             node_name='tello_driver', parameters=tello_driver_args),
        Node(package='tello_driver', node_executable='tello_joy_main', output='screen'),
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             node_name='vloc_node', parameters=[{
                'sub_camera_info_best_effort_not_reliable': 1,
            }]),
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             node_name='vmap_node', parameters=[{
                'use_sim_time': False,  # Use /clock if available
                'publish_tfs': 1,  # Publish marker /tf
                'marker_length': 0.1778,  # Marker length
                'marker_map_load_full_filename': map_path,  # Load a pre-built map from disk
                'make_not_use_map': 0  # Don't save a map to disk
            }]),
    ])
