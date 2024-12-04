import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('twist_out_cfs', default_value='rover_app_send_robot_command',
                          description='topic name for twist cfs cmd'),
    DeclareLaunchArgument('odom_in_cfs', default_value='rover_app_get_robot_odom',
                          description='topic name for odom cfs tlm'),
    DeclareLaunchArgument('twist_out', default_value='/cmd_vel',
                          description='topic name for twist topic to robot'),
    DeclareLaunchArgument('odom_in', default_value='/model/curiosity_mars_rover/odometry',
                          description='topic name for odom topic from robot'),                          
]

#####################################
def generate_launch_description():

  # Start the twist_odom_converter
  launch_mars_rover = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          PathJoinSubstitution([
              get_package_share_directory('mars_rover'), 
              'launch', 'mars_rover.launch.py'
          ])
      ])              
  )
  

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
          
  save_image = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
          PathJoinSubstitution([
              FindPackageShare('brash_application_tools'), 
              'launch', 'save_image_from_topic.launch.py'
          ])
      ]),
      launch_arguments = {
            'image_topic': '/image_raw',
            'image_folder': '/code/brash/cfdp/rosfsw',
            'image_name':  "rover_image.png",
            'image_size': '100',
            'timer_dt': '5.0'           
      }.items(),              
  )
            
          
  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(launch_mars_rover)
  ld.add_action(toc_convert)
  ld.add_action(save_image)
  return ld
  

