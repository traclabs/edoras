from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "example_arg",
            default_value='false',
            description="example arg to show how to do this",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='true',
            description="launch rviz",
        )
    )
    example_arg = LaunchConfiguration("example_arg")
    rviz = LaunchConfiguration("rviz")


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "spot_flight_demo.rviz"]
    )

    spot_launch = IncludeLaunchDescription(
       PythonLaunchDescriptionSource([os.path.join(
          get_package_share_directory('champ_bringup'), 'launch'),
          '/champ_bringup.launch.py']),
#      launch_arguments={'use_rviz': 'False', 'xyz': '0 0 0', 'rpy': '0 0 0'}.items()
      )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )


    nodes_to_start = [
        spot_launch,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


