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

  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'gateway_single_arm', 'ground_bridge.yaml')

  launch_args = [
      DeclareLaunchArgument("rviz", default_value="true"),
      DeclareLaunchArgument("bridge_config_file", default_value=config)
  ] 

  # ********************************
  # Conversion Bridge in Ground
  # ********************************
  edoras_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('conversion_tool'), 
         'launch', 'conversion.launch.py')]),
      launch_arguments={'config': LaunchConfiguration("bridge_config_file")}.items()
    )

  # **************************
  #  Big Arm
  # **************************
  big_arm_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gateway_description"), "robots", "big_arm.urdf.xacro"]),
        ])
  big_arm_robot_description = {"robot_description": big_arm_xacro}

  big_arm_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ground_rsp",
        remappings=[
          ('/ground/joint_states', '/telemetry_joint_state')
        ],
        parameters=[
            big_arm_robot_description,
            {'frame_prefix': 'ground/'}], # big_arm/
        namespace="ground",
        output="both",
  )

  rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "gateway_big_arm_ground_demo.rviz"]
  )

  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_ground",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("rviz"))
  )

  # ****************************************
  # Node to send commands using the gimbal
  # ****************************************  
  arm_command_node = Node(
          package='edoras_demos',
          executable='arm_ground_command',
          name='arm_ground_command',
          output='screen',
          parameters=[
            {'output_topic': 'command_pose'}],
          ) 

  return LaunchDescription(launch_args + 
    [
     edoras_bridge,
     big_arm_rsp,
     rviz_node,
     arm_command_node    
    ]
  )  
