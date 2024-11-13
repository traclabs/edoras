import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('port', default_value='9090',
                          description='port'),
    DeclareLaunchArgument('address', default_value=''),
    DeclareLaunchArgument('ssl', default_value='False'),
    DeclareLaunchArgument('certfile', default_value=''),
    DeclareLaunchArgument('keyfile', default_value=''),                              
    DeclareLaunchArgument('retry_startup_delay', default_value='5.0'),

    DeclareLaunchArgument('fragment_timeout',  default_value='600'),
    DeclareLaunchArgument('delay_between_messages',  default_value='0'),
    DeclareLaunchArgument('max_message_size',  default_value='10000000'),
    DeclareLaunchArgument('unregister_timeout',  default_value='10.0'),

    DeclareLaunchArgument('use_compression',  default_value='False'),
    DeclareLaunchArgument('call_services_in_new_thread',  default_value='False'),

    DeclareLaunchArgument('topics_glob',  default_value=''),
    DeclareLaunchArgument('services_glob',  default_value=''),
    DeclareLaunchArgument('params_glob',  default_value=''),
    DeclareLaunchArgument('bson_only_mode',  default_value='False')

#  <arg unless="$(var bson_only_mode)" name="binary_encoder" default="default"/>                                 
]

#####################################
def generate_launch_description():

  rosapi = Node(
          package='rosapi',
          executable='rosapi_node',
          name='rosapi',
          output='screen',
          parameters=[
            {"topics_glob": LaunchConfiguration("topics_glob")},
            {"services_glob": LaunchConfiguration("services_glob")},
            {"params_glob": LaunchConfiguration("params_glob")}
          ]) 


#  <group unless="$(var ssl)">
  rosbridge_node = Node(
          package='rosbridge_server',
          executable='rosbridge_websocket',
          name='rosbridge_websocket',
          output='screen',
          parameters=[
            {"port": LaunchConfiguration("port")},
            {"address": LaunchConfiguration("address")},
            {"retry_startup_delay": LaunchConfiguration("retry_startup_delay")},
            {"fragment_timeout": LaunchConfiguration("fragment_timeout")},
            {"delay_between_messages": LaunchConfiguration("delay_between_messages")},
            {"max_message_size": LaunchConfiguration("max_message_size")},
            {"unregister_timeout": LaunchConfiguration("unregister_timeout")},
            {"use_compression": LaunchConfiguration("use_compression")},
            {"call_services_in_new_thread": LaunchConfiguration("call_services_in_new_thread")},
            {"topics_glob": LaunchConfiguration("topics_glob")},
            {"services_glob": LaunchConfiguration("services_glob")},
            {"params_glob": LaunchConfiguration("params_glob")},
            {"bson_only_mode": LaunchConfiguration("bson_only_mode")}
          ]) 
#  </group>

#  <group if="$(var ssl)">
#    <node name="rosbridge_websocket" pkg="rosbridge_server" exec="rosbridge_websocket" output="screen">
#      <param name="certfile" value="$(var certfile)" />
#      <param name="keyfile" value="$(var keyfile)" />
#      <param name="port" value="$(var port)"/>
#      <param name="address" value="$(var address)"/>
#      <param name="retry_startup_delay" value="$(var retry_startup_delay)"/>
#      <param name="fragment_timeout" value="$(var fragment_timeout)"/>
#      <param name="delay_between_messages" value="$(var delay_between_messages)"/>
#      <param name="max_message_size" value="$(var max_message_size)"/>
#      <param name="unregister_timeout" value="$(var unregister_timeout)"/>
#      <param name="use_compression" value="$(var use_compression)"/>
#      <param name="call_services_in_new_thread" value="$(var call_services_in_new_thread)"/>

#      <param name="topics_glob" value="$(var topics_glob)"/>
#      <param name="services_glob" value="$(var services_glob)"/>
#      <param name="params_glob" value="$(var params_glob)"/>
#    </node>
#  </group>
          
  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(rosapi)
  ld.add_action(rosbridge_node)
  return ld
  
