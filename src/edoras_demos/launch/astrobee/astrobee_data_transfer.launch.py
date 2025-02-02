from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='true',
            description="launch rviz",
        )
    )

    rviz = LaunchConfiguration("rviz")

    
    # Edoras Bridge
    config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'astrobee', 'data_transfer.yaml')

    flight_data = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('data_transfer'), 
         'launch', 'flight_data_transfer.launch.py')]),
      launch_arguments={'config': config, 'use_sim_time': 'True'}.items()
    )
  

    ########################

    nodes_to_start = [
        flight_data
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


