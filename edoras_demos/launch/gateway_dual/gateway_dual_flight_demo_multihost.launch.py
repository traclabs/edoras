from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'gateway_dual', 'flight_bridge_multihost.yaml')

  dual_flight_demo_multihost = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('edoras_demos'), 
         'launch', 'gateway_dual', 'gateway_dual_flight_demo.launch.py')]),
      launch_arguments={'bridge_config_file': config,
                        'rviz': 'True'}.items()
  )  

  return LaunchDescription(
    [dual_flight_demo_multihost]
  )
