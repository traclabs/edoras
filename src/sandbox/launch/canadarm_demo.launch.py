from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory
import os
import xacro

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

    # **************************
    # Canadarm
    # **************************
    simulation_models_path = get_package_share_directory('simulation')
    canadarm_urdf_path = os.path.join(simulation_models_path, 'models', 'canadarm', 'urdf', 'SSRMS_Canadarm2.urdf.xacro')

    doc = xacro.process_file(canadarm_urdf_path, mappings={'xyz' : '1.0 0.0 1.5', 'rpy': '3.1416 0.0 0.0'})
        
    canadarm_robot_description = {"robot_description": doc.toxml()}

    canadarm_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            canadarm_robot_description,
            {'frame_prefix': ''}], # big_arm/
        namespace="canadarm"
    )
    canadarm_jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="canadarm"
    )



    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "canadarm_demo.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )

    # ***********************
    # Arm Comm
    # ***********************
    #robot_comm_node = Node(
    #    package="edoras_demos",
    #    executable="arm_comm_udp_node",
    #    name="arm_comm_udp_node",
    #    output="screen",
    #)    

    nodes_to_start = [
        #mars_surface_rsp,
        canadarm_rsp,
        canadarm_jsp,
        rviz_node,
        #robot_comm_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


