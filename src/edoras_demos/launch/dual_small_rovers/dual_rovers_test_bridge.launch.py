from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='true',
            description="launch rviz",
        )
    )

    rviz = LaunchConfiguration("rviz")

    # Rovers simulation
    # ros2 topic pub --rate 30 /robot_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    sim_rovers = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('edoras_demos'), 'launch', 'dual_small_rovers'),
         '/dual_rovers_gazebo_sim.launch.py']),
      launch_arguments={'rviz': 'false'}.items()
    )
    
    # Throttle: msgs_per_sec: 5
    throttle_flight_pose_1_node = Node(
        package="topic_tools",
        executable="throttle",
        name="throttle_flight_pose_1",
        output="screen",
        arguments=["messages", "/robot_1/pose", "5", "/robot_1/throttled_pose"]
    )     
    
    # Throttle: msgs_per_sec: 5
    throttle_flight_pose_2_node = Node(
        package="topic_tools",
        executable="throttle",
        name="throttle_flight_pose_2",
        output="screen",
        arguments=["messages", "/robot_2/pose", "5", "/robot_2/throttled_pose"]
    )     

    
    # Edoras Bridge
    config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'dual_small_rovers', 'flight_bridge.yaml')

    edoras_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('conversion_tool'), 
         'launch', 'flight_conversion.launch.py')]),
      launch_arguments={'config': config}.items()
    )
  

    ########################

    nodes_to_start = [
        #sim_rovers,
        #throttle_flight_pose_1_node,
        #throttle_flight_pose_2_node,        
        edoras_bridge
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


