cmake_minimum_required(VERSION 3.16)
project(edoras_demos)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  trac_ik_lib
  tf2_kdl
  interactive_markers
)

find_package(ament_cmake REQUIRED)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)


add_executable(arm_comm_udp_node 
  src/arm_comm_udp_node.cpp
  src/arm_comm_udp.cpp
  src/serialize_arm_manual.cpp
)
ament_target_dependencies(arm_comm_udp_node ${dependencies} )

add_executable(arm_comm_default_node 
  src/arm_comm_default_node.cpp
  src/arm_comm_default.cpp
)
ament_target_dependencies(arm_comm_default_node ${dependencies} )


add_executable(arm_ground_command 
  src/arm_ground_command.cpp
)
ament_target_dependencies(arm_ground_command ${dependencies} )

add_executable(rover_comm_udp_node
    src/rover_comm_udp_node.cpp 
    src/rover_comm_udp.cpp
    src/serialize_rover_manual.cpp
)
ament_target_dependencies(rover_comm_udp_node ${dependencies} )

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

add_executable(fake_base_cmd src/fake_base_cmd.cpp)
ament_target_dependencies(fake_base_cmd ${dependencies} )

ament_export_dependencies(
#  rclcpp
  ${dependencies}
)

install(TARGETS
  fake_base_cmd
  arm_ground_command
  arm_comm_udp_node
  rover_comm_udp_node
  arm_comm_default_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install folders
install(DIRECTORY
  launch
  rviz
  robots
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
