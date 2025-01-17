import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)

ARGUMENTS = [
    DeclareLaunchArgument('config', default_value='',
                          description='Configuration file for bridge')
]

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  conversion_node = Node(
          package='conversion_tool',
          executable='ground_conversion_node',
          name='conversion_node',
          output='screen',
          parameters=[LaunchConfiguration("config")]
          ) 

  # Request Telemetry back
  service_to = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " service call ",
            "/to_lab_enable_output_cmd ",
            'std_srvs/srv/SetBool ',
            '"{data: False}"',
        ]],
        shell=True
  )

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(conversion_node)
  ld.add_action( RegisterEventHandler(
                 OnProcessStart(
                 target_action=conversion_node,
                 on_start=[
                    LogInfo(msg='Edoras conversion node loaded. Calling TO service'),
                    #service_to,
                    TimerAction(
                        period=2.0,
                        actions=[service_to],
                    )
                ]
            )
        ) )
  return ld
  

