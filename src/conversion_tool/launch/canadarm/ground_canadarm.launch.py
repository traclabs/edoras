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

  # Robot description for the arm
  simulation_models_path = get_package_share_directory('simulation')
  urdf_model_path = os.path.join(simulation_models_path, 'models', 'canadarm', 'urdf', 'SSRMS_Canadarm2.urdf.xacro')
  doc = xacro.process_file(urdf_model_path, mappings={'xyz' : '1.0 0.0 1.5', 'rpy': '3.1416 0.0 0.0'})


  # Rviz visualization of arm
  rviz_base = os.path.join(get_package_share_directory("brash_application_tools"), "rviz")
  rviz_full_config = os.path.join(rviz_base, "canadarm.rviz")
  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_full_config]
  )


  rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
          {'robot_description': doc.toxml()}
        ],
      )

  # Start the joint_converter
  hk_node = Node(
        package="brash_application_tools",
        executable="canadarm_hk_joint_state_convert.py",
        name="canadarm_hk_joint_state_convert",
        output="screen"
  )
  
  # rosbridge_server for openmct
  bridge_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution(
        [get_package_share_directory("brash_application_tools"), 'launch', 'rosbridge_websocket.launch.py'])
        ]),
        launch_arguments = {
          'port': '9080',
          'address': '10.5.0.2'          
        }.items()
    )

  # Run openmct
  openmct_node = ExecuteProcess( 
    cmd = ['npm', 'start', '--prefix', "/code/openmct_ros"],
    shell = True
    )  
    
  
  return LaunchDescription(
      [
       rviz_node,
       hk_node,
       rsp,
       bridge_server,
       openmct_node
      ]
  )
