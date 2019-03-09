import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

sys.path.append(get_package_share_directory('fiducial_vlam') + '/launch/')
import dual_launch_two

def generate_launch_description():
    action_list = dual_launch_two.generate_main_action_list('drone1')
    return LaunchDescription(action_list)
