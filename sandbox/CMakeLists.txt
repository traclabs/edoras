cmake_minimum_required(VERSION 3.16)
project(sandbox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  dynmsg
  std_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

find_package(ament_cmake REQUIRED)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

add_executable(test_dynmsg_joint_states src/test_dynmsg_joint_states.cpp)
ament_target_dependencies(test_dynmsg_joint_states ${dependencies} )

add_executable(test_dynmsg_header src/test_dynmsg_header.cpp)
ament_target_dependencies(test_dynmsg_header ${dependencies} )


ament_export_dependencies(
#  rclcpp
  ${dependencies}
)

install(TARGETS
#  rover_comm_udp_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install folders
install(DIRECTORY
  launch
  rviz
  robots
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
