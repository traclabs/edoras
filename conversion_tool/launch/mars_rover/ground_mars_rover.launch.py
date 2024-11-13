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

  # rqt node - Send twist commands
  rqt_node = ExecuteProcess(
    cmd = ['rqt', '--perspective-file', os.path.join(get_package_share_directory("brash_application_tools"), "config", "mars_rover", "rover_app_mars.perspective")],
    shell = True
    )

  # Rviz visualization of odometry
  rviz_base = os.path.join(get_package_share_directory("brash_application_tools"), "rviz")
  rviz_full_config = os.path.join(rviz_base, "mars_rover.rviz")
  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_full_config]
  )

  # Start the twist_odom_converter
  launch_to_converter = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          PathJoinSubstitution([
              FindPackageShare('brash_application_tools'), 
              'launch', 'ground_twist_odom_convert.launch.py'
          ])
      ]),
      launch_arguments = {
            'odom_out': '/odom',
            'twist_in': '/cmd_vel'          
      }.items(),              
  )
  
  # rosbridge_server for openmct
  bridge_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution(
        [get_package_share_directory("brash_application_tools"), 'launch', 'rosbridge_websocket.launch.py'])
        ]),
        launch_arguments = {
          'port': '9080',
          'address': '10.5.0.2'          
        }.items()
    )
    
  # Run openmct
  openmct_node = ExecuteProcess( 
    cmd = ['npm', 'start', '--prefix', "/code/openmct_ros"],
    shell = True
    )  

  return LaunchDescription(
      [
       rqt_node,
       rviz_node,
       launch_to_converter,
       bridge_server,
       openmct_node
      ]
  )
