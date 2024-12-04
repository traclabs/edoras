import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('twist_cmd_cfs', default_value='rover_app_cmd',
                          description='topic name for twist cfs cmd'),
    DeclareLaunchArgument('odom_tlm_cfs', default_value='rover_app_hk_tlm',
                          description='topic name for odom cfs tlm'),
    DeclareLaunchArgument('odom_out', default_value='/odom',
                          description='topic name for twist topic to robot'),
    DeclareLaunchArgument('twist_in', default_value='/cmd_vel',
                          description='topic name for odom topic from robot'),                          
]

#####################################
def generate_launch_description():

  toc_convert = Node(
          package='brash_application_tools',
          executable='ground_twist_odom_convert.py',
          name='ground_twist_odom_convert',
          output='screen',
          parameters=[
            {"twist_cmd_cfs": LaunchConfiguration("twist_cmd_cfs")},
            {"odom_tlm_cfs": LaunchConfiguration("odom_tlm_cfs")},
            {"odom_out": LaunchConfiguration("odom_out")},
            {"twist_in": LaunchConfiguration("twist_in")} 
          ]) 
          
  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(toc_convert)
  return ld
  

