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
  
  # Send commands
  steering_1 = Node(
          package='rqt_robot_steering',
          executable='rqt_robot_steering',
          name='steering_1',
          output='screen',
          namespace='robot_1/ground'
          )

  steering_2 = Node(
          package='rqt_robot_steering',
          executable='rqt_robot_steering',
          name='steering_2',
          output='screen',
          namespace='robot_2/ground'
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
        parameters=[{'use_sim_time': use_sim_time}],
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
      launch_arguments={'config': config,
                        'use_sim_time': use_sim_time}.items()
  )

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(steering_1)
  ld.add_action(steering_2)
  #ld.add_action(throttle_ground_twist_1_node)
  #ld.add_action(throttle_ground_twist_2_node)  
  ld.add_action(rviz_node)
  ld.add_action(edoras_bridge)
  return ld
  

