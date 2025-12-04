import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

#####################################
def generate_launch_description():

  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'gateway_single_arm', 'ground_bridge_multihost.yaml')

  single_ground_demo_multihost = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('edoras_demos'), 
         'launch', 'gateway_single_arm', 'gateway_single_arm_ground_demo.launch.py')]),
      launch_arguments={'bridge_config_file': config,
                        'rviz': 'True'}.items()
  )  

  return LaunchDescription(
    [single_ground_demo_multihost]
  )
  
  return ld
  

