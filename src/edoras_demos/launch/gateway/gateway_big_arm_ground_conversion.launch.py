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
]

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'gateway', 'ground_bridge.yaml')
  conversion_node = Node(
          package='conversion_tool',
          executable='ground_conversion_node',
          name='ground_conversion_node',
          output='screen',
          parameters=[config]
          ) 

  #test_publish_node = Node(
  #        package='conversion_tool',
  #        executable='test_publisher_rover',
  #        name='test_publisher_rover',
  #        output='screen'
  #        ) 

  #steering_node = Node(
  #        package='rqt_robot_steering',
  #        executable='rqt_robot_steering',
  #        name='send_robot_steering',
  #        output='screen'
  #        )

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(conversion_node)
  #ld.add_action(test_publish_node)
  #ld.add_action(steering_node)
  return ld
  

