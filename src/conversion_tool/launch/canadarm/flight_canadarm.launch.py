import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

#####################################
def generate_launch_description():

  # Start the twist_odom_converter
  launch_canadarm = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          PathJoinSubstitution([
              get_package_share_directory('canadarm'), 
              'launch', 'canadarm.launch.py'
          ])
      ])              
  )
  
  # Launch action node
  canadarm_control = Node(
          package='brash_application_tools',
          executable='canadarm_control_node.py',
          name='canadarm_control_node',
          output='screen',
          ) 
          
  ld = LaunchDescription()
  ld.add_action(launch_canadarm)
  ld.add_action(canadarm_control)
  return ld
  

