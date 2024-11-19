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
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(rclcpp REQUIRED)
find_package(edoras_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(SRC_FILES
  src/ground_conversion.cpp
)

add_library(ground_conversion SHARED ${SRC_FILES})
ament_target_dependencies(ground_conversion
  rclcpp
  ${dependencies}
)

add_executable(test_publisher tools/test_publisher.cpp)
ament_target_dependencies(test_publisher
  rclcpp
  ${dependencies}
)

add_executable(test_conversion_tool tools/test_conversion_tool.cpp)
ament_target_dependencies(test_conversion_tool
  rclcpp
  ${dependencies}
)
target_link_libraries(test_conversion_tool ground_conversion)

ament_export_dependencies(
  rclcpp
  rclpy
  ${dependencies}
)


install(TARGETS
  ground_conversion 
  test_publisher
  test_conversion_tool
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
