from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

  rover_flight_demo_multihost = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('edoras_demos'), 
         'launch', 'rover', 'rover_flight_demo.launch.py')]),
      launch_arguments={'cfs_ip': "10.5.0.3",
                        'robot_ip': "10.5.0.4"}.items()
  )  

  return LaunchDescription(
    [rover_flight_demo_multihost]
  )


