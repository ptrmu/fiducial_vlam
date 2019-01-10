import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Full teleop launch: driver, base, joystick, etc.


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('flock2'), 'urdf', 'tello.urdf')
    return LaunchDescription([
        ExecuteProcess(cmd=['rviz2', '-d', 'src/flock_vlam/flock_vlam/cfg/default.rviz'], output='screen'),
        Node(package='tello_driver', node_executable='tello_driver', output='screen'),
        Node(package='tello_driver', node_executable='tello_joy', output='screen'),
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='flock_vlam', node_executable='vloc_node', output='screen'),
        Node(package='flock_vlam', node_executable='vmap_node', output='screen'),
    ])
