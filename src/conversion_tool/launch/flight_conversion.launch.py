import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)

ARGUMENTS = [
    DeclareLaunchArgument('config', default_value='',
                          description='Configuration file for bridge')
]

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  flight_conversion_node = Node(
          package='conversion_tool',
          executable='flight_conversion_node',
          name='flight_conversion_node',
          output='screen',
          parameters=[LaunchConfiguration("config")]
          ) 

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(flight_conversion_node)

  return ld
  

