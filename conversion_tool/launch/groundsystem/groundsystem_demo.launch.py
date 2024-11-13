import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro

#####################################
def generate_launch_description():
    
  # rqt node
  rqt_node = ExecuteProcess(
    cmd = ['rqt', '--perspective-file', os.path.join(get_package_share_directory("brash_application_tools"), "config", "groundsystem", "groundsystem.perspective")],
    shell = True
    )
  

  return LaunchDescription(
      [
       rqt_node
      ]
  )
