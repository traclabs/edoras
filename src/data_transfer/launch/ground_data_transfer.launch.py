import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)

ARGUMENTS = [
    DeclareLaunchArgument('config', default_value='',
                          description='Configuration file for data_transfer'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          description='Configuration file for bridge')                          
]

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  # Config
  config = LaunchConfiguration("config")
  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'dual_small_rovers', 'ground_data_transfer.yaml')

  data_transfer_node = Node(
          package='data_transfer',
          executable='data_transfer_node',
          name='data_transfer_node',
          output='screen',
          parameters=[config,
          {'use_sim_time': LaunchConfiguration("use_sim_time")}]
          ) 

  ground_cfdp_transfer = Node(
          package='data_transfer',
          executable='ground_cfdp_transfer.py',
          name='ground_cfdp_transfer',
          output='screen',
          parameters=[config,
          {'use_sim_time': LaunchConfiguration("use_sim_time")}]
          ) 


  ld = LaunchDescription(ARGUMENTS)
  #ld.add_action(data_transfer_node)
  ld.add_action(ground_cfdp_transfer)
  return ld
  

