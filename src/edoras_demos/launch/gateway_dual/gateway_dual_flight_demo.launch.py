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

    config = os.path.join(get_package_share_directory('edoras_demos'), 'config', 'gateway_dual', 'flight_bridge.yaml')

    launch_args = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("bridge_config_file", default_value=config)
    ]    

    # *****************************
    # Gateway body (static)
    # *****************************
    gateway_body_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gateway_description"), "robots", "gateway_body.urdf.xacro"]),
        ])
    gateway_robot_description = {"robot_description": gateway_body_xacro}

    gateway_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            gateway_robot_description,
            {'frame_prefix': ''}], # gateway/
        namespace="gateway_body"
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
        output="both",
        parameters=[
            big_arm_robot_description,
            {'frame_prefix': ''}], # big_arm/
        namespace="big_arm"
    )
    big_arm_jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="big_arm",
        parameters=[
         {'source_list': ['joint_state_command'] }]
    )

    # *************************
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
        output="both",
        parameters=[
            little_arm_robot_description,
            {'frame_prefix': ''}], # little_arm/
        namespace="little_arm"
    )
    little_arm_jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="little_arm",
        parameters=[
        {'source_list': ['joint_state_command'] }]
    )


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "gateway_big_arm_flight_demo.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_flight",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    # ***********************
    # Arms Control
    # ***********************
    big_arm_ik_node = Node(
        package="edoras_demos",
        executable="arm_comm_default_node",
        name="arm_ik_node",
        parameters=[
          {"robot_description": big_arm_xacro,
          "joint_state": "/big_arm/joint_states",
          "joint_state_throttled": "/big_arm/joint_states_throttled",
          "joint_command": "/big_arm/joint_state_command",
          "target_pose": "/big_arm/flight/target_pose",
          "base_link": "big_arm_link_1",
          "tip_link": "big_arm_link_8"
          },
        ],
        output="screen",
        namespace="big_arm"
    )    

    little_arm_ik_node = Node(
        package="edoras_demos",
        executable="arm_comm_default_node",
        name="arm_ik_node",
        parameters=[
          {"robot_description": little_arm_xacro,
          "joint_state": "/little_arm/joint_states",
          "joint_state_throttled": "/little_arm/joint_states_throttled",
          "joint_command": "/little_arm/joint_state_command",          
          "target_pose": "/little_arm/flight/target_pose",
          "base_link": "little_arm_link_1",
          "tip_link": "little_arm_link_9"
          },
        ],
        output="screen",
        namespace="little_arm"
    )    

    # ***********************
    # Edoras Bridge
    # ***********************
    edoras_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('conversion_tool'), 
         'launch', 'flight_conversion.launch.py')]),
      launch_arguments={'config': LaunchConfiguration("bridge_config_file"), 
                        'use_sim_time': 'False'}.items()
    )

    nodes_to_start = [
        gateway_rsp,
        big_arm_rsp,
        big_arm_jsp,
        little_arm_rsp,
        little_arm_jsp,
        rviz_node,
        big_arm_ik_node,
        little_arm_ik_node,
        edoras_bridge
    ]

    return LaunchDescription(launch_args + nodes_to_start)


