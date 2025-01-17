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
    sim_rovers = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('edoras_demos'), 'launch', 'dual_small_rovers'),
         '/dual_rovers_gazebo_sim.launch.py']),
    #  launch_arguments={'ns': 'panda_2/', 'xyz': '1 0 0', 'rpy': '0 0 3.1416'}.items()
    )
    
    # Bridge 1
    config_1 = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'dual_small_rovers', 'flight_bridge_1.yaml')
  
    bridge_1 = Node(
          package='conversion_tool',
          executable='ground_conversion_node',
          name='ground_conversion_node',
          output='screen',
          ns='robot_1',
          parameters=[config_1]
    ) 

    ########################

    nodes_to_start = [
        sim_rovers,
        bridge_1,
        bridge_2
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


