cmake_minimum_required(VERSION 3.5)
project(conversion_tool)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
edoras_msgs
rviz_common
rviz_rendering)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(rclcpp REQUIRED)
#find_package(pluginlib REQUIRED)
#find_package(Qt5 REQUIRED COMPONENTS Widgets)
#find_package(rviz_common REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(cfe_msgs REQUIRED)
#find_package(cfdp_msgs REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

#include_directories(include)

#set(SRC_FILES
#  src/canadarm_panel.cpp
#  src/mars_rover_panel.cpp
#)

#add_library(${PROJECT_NAME}_rviz_plugins SHARED ${SRC_FILES})
#ament_target_dependencies(${PROJECT_NAME}_rviz_plugins
#  rclcpp
#  ${dependencies}
#)
#target_include_directories(${PROJECT_NAME}_rviz_plugins PUBLIC
#  ${Qt5Widgets_INCLUDE_DIRS})

## Here we export the plugins and meshes so they can be found by RViz at runtime.
#pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(
  rclcpp
  rclpy
  ${dependencies}
)



#install(TARGETS
#  ${PROJECT_NAME}_rviz_plugins
#  DESTINATION lib/${PROJECT_NAME}
#)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/parse_edoras_msgs.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders
#install(DIRECTORY
#  config launch meshes resource rviz urdf worlds
#  DESTINATION share/${PROJECT_NAME}/
#)


ament_package()
