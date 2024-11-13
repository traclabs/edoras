import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('twist_out_cfs', default_value='rover_app_send_robot_command',
                          description='topic name for twist cfs cmd'),
    DeclareLaunchArgument('odom_in_cfs', default_value='rover_app_get_robot_odom',
                          description='topic name for odom cfs tlm'),
    DeclareLaunchArgument('twist_out', default_value='/cmd_vel',
                          description='topic name for twist topic to robot'),
    DeclareLaunchArgument('odom_in', default_value='/odom',
                          description='topic name for odom topic from robot'),                          
]

#####################################
def generate_launch_description():

  toc_convert = Node(
          package='brash_application_tools',
          executable='flight_twist_odom_convert.py',
          name='flight_twist_odom_convert',
          output='screen',
          parameters=[
            {"twist_out_cfs": LaunchConfiguration('twist_out_cfs')},
            {"odom_in_cfs": LaunchConfiguration('odom_in_cfs')},
            {"odom_in": LaunchConfiguration('odom_in')},
            {"twist_out": LaunchConfiguration('twist_out')} 
          ]) 
          
  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(toc_convert)
  return ld
  

