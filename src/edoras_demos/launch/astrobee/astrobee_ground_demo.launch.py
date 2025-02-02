import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          description='rviz in Ground'),
    DeclareLaunchArgument('use_sim_time', default_value='True',
                          description='use_sim_time true if gazebo'),
]

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  rviz = LaunchConfiguration("rviz")
  use_sim_time = LaunchConfiguration("use_sim_time")
        
          
  # View telemetry back
  rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "astrobee_ground_demo.rviz"]
  )

  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_ground",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
  )  
    

  # Edoras Bridge
  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'astrobee', 'ground_data_transfer.yaml')

  ground_transfer = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('data_transfer'), 
         'launch', 'ground_data_transfer.launch.py')]),
      launch_arguments={'config': config,
                        'use_sim_time': use_sim_time}.items()
  )

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(rviz_node)
  ld.add_action(ground_transfer)
  return ld
  

