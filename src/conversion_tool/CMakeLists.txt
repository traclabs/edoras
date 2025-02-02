cmake_minimum_required(VERSION 3.5)
project(conversion_tool)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  edoras_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()


include_directories(include)

# basic_communication -------------
add_library(basic_communication src/basic_communication.cpp)

# ground_conversion -------------
add_library(ground_conversion SHARED 
            src/ground_conversion.cpp
            src/parser_utils.cpp
            src/debug_utils.cpp
            src/memory_utils.cpp)
ament_target_dependencies(ground_conversion
  rclcpp
  ${dependencies}
)
target_link_libraries(ground_conversion basic_communication)

# flight_conversion -------------
add_library(flight_conversion SHARED 
            src/flight_conversion.cpp
            src/peer_communication.cpp
            src/sbn_peer.cpp
            src/memory_utils.cpp
            src/parser_utils.cpp
            src/debug_utils.cpp)
ament_target_dependencies(flight_conversion
  rclcpp
  ${dependencies}
)

# Tools ***************************************
add_executable(test_publisher tools/test_publisher.cpp)
ament_target_dependencies(test_publisher
  rclcpp
  ${dependencies}
)

add_executable(test_publisher_rover tools/test_publisher_rover.cpp)
ament_target_dependencies(test_publisher_rover
  rclcpp
  ${dependencies}
)
# Tools ***************************************

add_executable(ground_conversion_node src/ground_conversion_node.cpp)
ament_target_dependencies(ground_conversion_node
  rclcpp
  ${dependencies}
)
target_link_libraries(ground_conversion_node ground_conversion)

add_executable(flight_conversion_node src/flight_conversion_node.cpp)
ament_target_dependencies(flight_conversion_node
  rclcpp
  ${dependencies}
)
target_link_libraries(flight_conversion_node flight_conversion)


ament_export_dependencies(
  rclcpp
  rclpy
  ${dependencies}
)


install(TARGETS
  basic_communication
  ground_conversion
  flight_conversion
  test_publisher
  test_publisher_rover
  ground_conversion_node
  flight_conversion_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/parse_edoras_msgs.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
