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
    DeclareLaunchArgument('odom_in_cfs', default_value='rover_app_get_robot_odom',
                          description='topic name for odom cfs tlm'),
]

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  rviz = LaunchConfiguration("rviz")
  
  # Send commands
  steering_node_1 = Node(
          package='rqt_robot_steering',
          executable='rqt_robot_steering',
          name='send_robot_steering',
          output='screen',
          namespace='robot_1_ground'
          )
  # View telemetry back
  rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "dual_small_rovers_ground_demo.rviz"]
  )

  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_ground",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
  )  

  # Throttle: msgs_per_sec: 5
  throttle_ground_twist_1_node = Node(
        package="topic_tools",
        executable="throttle",
        name="throttle_ground_twist_1",
        output="screen",
        arguments=["messages", "/robot_1/cmd_vel", "5", "/robot_1/throttled_cmd_vel"]
  )     
    
  # Throttle: msgs_per_sec: 5
  throttle_ground_twist_2_node = Node(
        package="topic_tools",
        executable="throttle",
        name="throttle_flight_pose_2",
        output="screen",
        arguments=["messages", "/robot_2/cmd_vel", "5", "/robot_2/throttled_cmd_vel"]
  )     



  # Edoras Bridge
  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'dual_small_rovers', 'ground_bridge.yaml')

  edoras_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('conversion_tool'), 
         'launch', 'conversion.launch.py')]),
      launch_arguments={'config': config}.items()
  )

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(steering_node_1)
  ld.add_action(throttle_ground_twist_1_node)
  ld.add_action(throttle_ground_twist_2_node)  
  ld.add_action(rviz_node)
  ld.add_action(edoras_bridge)
  return ld
  

