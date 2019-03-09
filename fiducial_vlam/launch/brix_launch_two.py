import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import dual_launch_two

def generate_launch_description():
    action_list = dual_launch_two.generate_one_drone_action_list('drone2')
    return LaunchDescription(action_list)
