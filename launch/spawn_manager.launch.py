'''Launch ur10 ignition_simulator'''

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld=LaunchDescription()
    # ign_gazebo_manager 
    gz_gazebo_manager = Node(package='ros_gz_gazebo_manager',
            executable='gz_gazebo_manager',
            name="gz_gazebo_manager",
            output='screen'
    )
    ld.add_action(gz_gazebo_manager)
    return ld
