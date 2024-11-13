import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('image_topic', default_value='/image_raw'),
    DeclareLaunchArgument('image_folder', default_value='/code/brash/cfdp/rosfsw',
                          description='w.r.t. brash workspace'),
    DeclareLaunchArgument('image_name', default_value='rover_image.png'),
    DeclareLaunchArgument('timer_dt', default_value='5.0'),
    DeclareLaunchArgument('image_size', default_value='100'),    
]

#####################################
def generate_launch_description():

  save_image = Node(
          package='brash_application_tools',
          executable='save_image_from_topic.py',
          name='save_image_from_topic',
          output='screen',
          parameters=[
            {"image_topic": LaunchConfiguration('image_topic')},
            {"image_folder": LaunchConfiguration('image_folder')},
            {"image_name": LaunchConfiguration('image_name')},
            {"timer_dt": LaunchConfiguration('timer_dt')},
            {"image_size": LaunchConfiguration('image_size')}             
          ]) 
          
  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(save_image)
  return ld
  

