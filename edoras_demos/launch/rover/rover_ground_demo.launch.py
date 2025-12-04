import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'rover', 'ground_bridge.yaml')

  launch_args = [
      DeclareLaunchArgument("rviz", default_value="true"),
      DeclareLaunchArgument("bridge_config_file", default_value=config)
  ] 

  # Edoras Bridge
  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'rover', 'ground_bridge_multihost.yaml')
  edoras_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('conversion_tool'), 
         'launch', 'conversion.launch.py')]),
      launch_arguments={'config': LaunchConfiguration("bridge_config_file")}.items()
    )  
  
  # Send commands
  steering_node = Node(
          package='rqt_robot_steering',
          executable='rqt_robot_steering',
          name='send_robot_steering',
          output='screen'
          )
  # View telemetry back
  rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "rover_ground_demo.rviz"]
  )

  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_ground",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("rviz"))
  )  

  return LaunchDescription(launch_args + 
    [
     edoras_bridge, 
     steering_node, 
     rviz_node
    ]
  )
