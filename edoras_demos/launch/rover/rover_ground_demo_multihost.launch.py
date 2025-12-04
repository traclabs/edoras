import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'rover', 'ground_bridge_multihost.yaml')

  dual_flight_demo_multihost = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('edoras_demos'), 
         'launch', 'rover', 'rover_ground_demo.launch.py')]),
      launch_arguments={'bridge_config_file': config,
                        'rviz': 'True'}.items()
  )  

  return LaunchDescription(
    [dual_flight_demo_multihost]
  )

