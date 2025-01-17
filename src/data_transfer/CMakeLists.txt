cmake_minimum_required(VERSION 3.5)
project(data_transfer)

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
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()


include_directories(include)


# ground_conversion -------------
add_library(data_transfer SHARED 
            src/data_transfer.cpp)
ament_target_dependencies(data_transfer
  rclcpp
  ${dependencies}
)
#target_link_libraries(data_transfer xxx)

# Tools ***************************************
#add_executable(test_publisher tools/test_publisher.cpp)
#ament_target_dependencies(test_publisher
#  rclcpp
#  ${dependencies}
#)

#add_executable(test_publisher_rover tools/test_publisher_rover.cpp)
#ament_target_dependencies(test_publisher_rover
#  rclcpp
#  ${dependencies}
#)
# Tools ***************************************

#add_executable(ground_conversion_node src/ground_conversion_node.cpp)
#ament_target_dependencies(ground_conversion_node
#  rclcpp
#  ${dependencies}
#)
#target_link_libraries(ground_conversion_node ground_conversion)

ament_export_dependencies(
  rclcpp
  rclpy
  ${dependencies}
)


install(TARGETS
  data_transfer
  data_transfer_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  scripts/parse_edoras_msgs.py
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install folders
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
