cmake_minimum_required(VERSION 3.5)
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
)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(serialize_manual SHARED src/serialize_manual.cpp)
ament_target_dependencies(serialize_manual
  ${dependencies}
)

add_library(arm_comm_udp SHARED src/arm_comm_udp.cpp)
ament_target_dependencies(arm_comm_udp
  ${dependencies}
)
target_link_libraries(arm_comm_udp serialize_manual)


add_executable(arm_comm_udp_node src/arm_comm_udp_node.cpp)
ament_target_dependencies(arm_comm_udp_node ${dependencies} )
target_link_libraries(arm_comm_udp_node arm_comm_udp)

ament_export_dependencies(
#  rclcpp
  ${dependencies}
)

install(TARGETS
  serialize_manual
  arm_comm_udp
  arm_comm_udp_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install folders
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
