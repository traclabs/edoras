import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro

#####################################
def generate_launch_description():

  simulation_models_path = get_package_share_directory('simulation')

  urdf_model_path = os.path.join(simulation_models_path, 'models', 'canadarm', 'urdf', 'SSRMS_Canadarm2.urdf.xacro')
  doc = xacro.process_file(urdf_model_path, mappings={'xyz' : '1.0 0.0 1.5', 'rpy': '3.1416 0.0 0.0'})
  robot_description = {'robot_description': doc.toxml()}

  canadarm_rsp = Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[robot_description]) 

  rviz_base = os.path.join(get_package_share_directory("brash_application_tools"), "rviz")
  rviz_full_config = os.path.join(rviz_base, "canadarm.rviz")
  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_full_config]
  )

    
  # Joint State publisher
  joint_publisher = Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      name='joint_state_publisher_gui',
      remappings={('joint_states', "joint_command")},
      output='screen')

  # rqt node
  rqt_node = ExecuteProcess(
    cmd = ['rqt', '--perspective-file', os.path.join(get_package_share_directory("brash_demo"), "config", "robot_sim_canadarm.perspective")],
    shell = True
    )

  js_converter = Node(
          package='brash_application_tools',
          executable='joint_state_convert.py',
          name='joint_state_convert',
          output='screen',
          parameters=[
            {"joint_names": ["Base_Joint", "Shoulder_Roll", "Shoulder_Yaw", "Elbow_Pitch", "Wrist_Pitch", "Wrist_Yaw", "Wrist_Roll"]} 
          ]) 
  

  return LaunchDescription(
      [
       canadarm_rsp,
       rviz_node,
       joint_publisher,
       rqt_node,
       js_converter
      ]
  )
