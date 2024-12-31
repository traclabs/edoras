from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

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

    # *****************************
    # Mars surface
    # *****************************
    mars_surface_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("simulation"), 'models', 'curiosity_path',
            'urdf', 'curiosity_mars_rover.xacro']),
        ])
    mars_surface_robot_description = {"robot_description": mars_surface_xacro}

    mars_surface_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            mars_surface_robot_description,
            {'frame_prefix': ''}], # mars_surface/
        namespace="mars_surface"
    )

    # **************************
    # Mars Rover
    # **************************
    mars_rover_xacro = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("simulation"), 'models', 'curiosity_path',
            'urdf', 'curiosity_mars_rover.xacro']),
        ])
    mars_rover_robot_description = {"robot_description": mars_rover_xacro}

    mars_rover_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            mars_rover_robot_description,
            {'frame_prefix': ''}], # big_arm/
        namespace="mars_rover"
    )
    mars_rover_jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="mars_rover"
    )



    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("edoras_demos"), "rviz", "mars_rover_demo.rviz"]
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
        mars_rover_rsp,
        mars_rover_jsp,
        rviz_node,
        #robot_comm_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


