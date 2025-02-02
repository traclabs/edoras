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
                          description='Open rviz to see robot state back'),
    DeclareLaunchArgument('odom_in_cfs', default_value='rover_app_get_robot_odom',
                          description='topic name for odom cfs tlm'),
]

# If you want to use ros2 node list, sometimes the nodes do not appear
# https://github.com/ros2/ros2cli/issues/582
# short try, stopping and restarting the daemon: ros2 daemon stop && ros2 daemon start

#####################################
def generate_launch_description():

  rviz = LaunchConfiguration("rviz")

  # ********************************
  # Conversion Bridge in Ground
  # ********************************
  config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'gateway_dual', 'ground_bridge.yaml')
  
  edoras_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('conversion_tool'), 
         'launch', 'conversion.launch.py')]),
      launch_arguments={'config': config,
                        'use_sim_time': 'False'}.items()
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
          ('/big_arm/joint_states', '/big_arm/ground/joint_states')
        ],
        parameters=[
            big_arm_robot_description,
            {'frame_prefix': 'ba/'}], # big_arm/
        namespace="big_arm",
        output="both",
  )

  # **************************
  #  Little Arm
  # **************************
  little_arm_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gateway_description"), "robots", "little_arm.urdf.xacro"]),
        ])
  little_arm_robot_description = {"robot_description": little_arm_xacro}

  little_arm_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ground_rsp",
        remappings=[
          ('/little_arm/joint_states', '/little_arm/ground/joint_states')
        ],
        parameters=[
            little_arm_robot_description,
            {'frame_prefix': 'la/'}], # little_arm/
        namespace="little_arm",
        output="both",
  )


  rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "gateway_dual_ground_demo.rviz"]
  )

  rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_ground",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
  )

  big_arm_command = Node(
          package='edoras_demos',
          executable='arm_ground_command',
          name='ground_command',
          output='screen',
          parameters=[{
            'reference_frame': 'ba/big_arm_link_1',
            'output_topic': '/big_arm/ground/target' 
          }],
          namespace="big_arm"
          ) 

  little_arm_command = Node(
          package='edoras_demos',
          executable='arm_ground_command',
          name='ground_command',
          output='screen',
          parameters=[{
            'reference_frame': 'la/little_arm_link_1',
            'output_topic': '/little_arm/ground/target' 
          }],
          namespace="little_arm"
          ) 


  arms_tf = Node(
                  package='tf2_ros',
                  executable='static_transform_publisher',
                  name='ground_base_tf_node',
                  output='screen',
                  arguments=["--frame-id", "ba/root", "--child-frame-id", "la/root", 
                  "--x", "0.0", "--y", "12.0", "--z", "9.0",
                  "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0"])


  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(edoras_bridge)
  ld.add_action(big_arm_rsp)
  ld.add_action(little_arm_rsp)  
  ld.add_action(rviz_node)
  ld.add_action(little_arm_command)
  ld.add_action(big_arm_command)
  ld.add_action(arms_tf)
  return ld
  

