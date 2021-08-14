
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    keyboard_launch_file_dir = os.path.join(get_package_share_directory('py'), 'launch')
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([keyboard_launch_file_dir, '/keyboard.launch.py'])
        ),


    ])

