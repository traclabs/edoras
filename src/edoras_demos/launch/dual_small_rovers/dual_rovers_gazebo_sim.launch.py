import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():

    edoras_demos_share_dir = get_package_share_directory("edoras_demos")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_filepath = os.path.join(get_package_share_directory(
        'edoras_demos'), 'robots', 'rover_4wd.urdf.xacro')
    world = LaunchConfiguration('world')

    #robot_desc = ParameterValue(Command(['xacro ', urdf]),
    #                                   value_type=str)
    
    robot_desc_1 = xacro.process_file(
      xacro_filepath, 
      mappings={"robot_name": "robot_1"}
    ).toprettyxml(indent="  ")

    robot_desc_2 = xacro.process_file(
      xacro_filepath, 
      mappings={"robot_name": "robot_2"}
    ).toprettyxml(indent="  ")    
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='rocky_surface.sdf',
        description='World file to use in Gazebo')
    
    gz_world_arg = PathJoinSubstitution([
        get_package_share_directory('edoras_demos'), 'worlds', world])

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : gz_world_arg 
        }.items()
    )
    
    # Spawn Rover Robot 1
    gz_spawn_entity_1 = Node(
        package="ros_gz_sim",
        executable="create",
        namespace="robot_1",
        arguments=[
            "-topic", "/robot_1/robot_description",
            "-name", "robot_1",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ]
    )

    # Robot state publisher 1
    rsp_1 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            output='screen',
            parameters=[{
              'use_sim_time': use_sim_time,
              'robot_description': robot_desc_1,
              'frame_prefix': 'robot_1/'}],
            arguments=[],
            namespace="robot_1")


    # Spawn Rover Robot 2
    gz_spawn_entity_2 = Node(
        package="ros_gz_sim",
        executable="create",
        namespace="robot_2",
        arguments=[
            "-topic", "/robot_2/robot_description",
            "-name", "robot_2",
            "-allow_renaming", "true",
            "-x", "-0.5",
            "-y", "1.5",
            "-z", "0.1",
        ]
    )
    
    # Robot state publisher 2
    rsp_2 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            output='screen',
            parameters=[{
              'use_sim_time': use_sim_time,
              'robot_description': robot_desc_2,
              'frame_prefix': 'robot_2/'}],
            arguments=[],
            namespace="robot_2")
    
    
    bridge_config_file = os.path.join(
        edoras_demos_share_dir, 'config', "dual_small_rovers", "gz_bridge.yaml")


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_file}],
        #arguments=[
        #    "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
        #    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        #    "/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
        #    "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        #    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        #    '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        #    '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        #],
        output='screen'
    )    

    # Rviz visualization of arm
    rviz_config = os.path.join(get_package_share_directory("edoras_demos"), "rviz", "dual_small_rovers.rviz")
    rviz_node = Node(
         package="rviz2",
         executable="rviz2",
         name="rviz2",
         output="screen",
         arguments=["-d", rviz_config]
    )
    

    return LaunchDescription([
      declare_use_sim_time_cmd,
      declare_world_cmd,
      gz_sim,
      gz_spawn_entity_1,
      gz_spawn_entity_2,
      bridge,
      rsp_1,
      rsp_2,
      rviz_node
    ])
