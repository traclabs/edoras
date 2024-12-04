import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

#####################################
def generate_launch_description():

  # rqt node
  rqt_node = ExecuteProcess(
    cmd = ['rqt', '--perspective-file', os.path.join(get_package_share_directory("brash_application_tools"), "config", "tracarm", "robot_sim_tracarm_groundsystem.perspective")],
    shell = True
    )


  # Launch tracarm 
  launch_tracarm = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution(
      [FindPackageShare("brash_application_tools"), 'launch', 'tracarm', 'bringup_tracarm.launch.py'])))


  # Rviz visualization of tracarm
  rviz_base = os.path.join(get_package_share_directory("brash_application_tools"), "rviz")
  rviz_full_config = os.path.join(rviz_base, "tracarm.rviz")
  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_full_config]
  )

    
  # Send commands - Joint State publisher
  joint_publisher = Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      name='joint_state_publisher_gui',
      parameters=[{"rate": 10}],
      remappings={('joint_states', "joint_command")},
      output='screen')

  # Start the joint state converter
  js_converter = Node(
          package='brash_application_tools',
          executable='joint_state_convert.py',
          name='joint_state_convert',
          output='screen',
          parameters=[
            {"joint_names": ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]} 
          ])   
  
  return LaunchDescription(
      [
       rqt_node,
       launch_tracarm,
       rviz_node,
       joint_publisher,
       js_converter
      ]
  )
