cmake_minimum_required(VERSION 3.5)
project(edoras_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(rosidl_default_generators REQUIRED)

#find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

FILE(GLOB MsgDefs msg/*.msg)
# message(".msg Files: ${MsgDefs}")
# Convert the absolute paths to relative paths
foreach(MsgDef ${MsgDefs})
    file(RELATIVE_PATH MsgDefRelative ${CMAKE_CURRENT_SOURCE_DIR} ${MsgDef})
    list(APPEND MsgDefsRelative ${MsgDefRelative})
endforeach()

# Now MsgDefsRelative contains the relative paths
message(STATUS "Relative MsgDefs: ${MsgDefsRelative}")

rosidl_generate_interfaces(${PROJECT_NAME} ${MsgDefsRelative}
#  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
   ament_lint_auto_find_test_dependencies()
endif()

ament_package()
