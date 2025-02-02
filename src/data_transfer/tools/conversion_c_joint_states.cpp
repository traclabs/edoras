//#include <yaml-cpp/yaml.h>
#include <string>

//#include <dynmsg/message_reading.hpp>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/header.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <dynmsg/typesupport.hpp>
#include <dynmsg/message_reading.hpp>
#include <iostream>
int main()
{

    sensor_msgs__msg__JointState* msg = sensor_msgs__msg__JointState__create();
    msg->header.stamp.sec = 4;
    msg->header.stamp.nanosec = 20u;
    rosidl_runtime_c__String__assign(&msg->header.frame_id, "my_frame");
    rosidl_runtime_c__double__Sequence__init(&msg->position, 4);

    msg->position.data[0] = 0.3;
    msg->position.data[1] = 0.4; 
    msg->position.data[2] = 0.6;

    rosidl_runtime_c__String__Sequence__init(&msg->name, 4);
    rosidl_runtime_c__String__assign(&msg->name.data[0], "mads");
    rosidl_runtime_c__String__assign(&msg->name.data[0], "ana");
    rosidl_runtime_c__String__assign(&msg->name.data[0], "debby");
    rosidl_runtime_c__String__assign(&msg->name.data[0], "dolly");

    // Convert into YAML
    //RosMessage_C ros_msg;

  std::cout << "Create ros message...." << std::endl;
  RosMessage message;
  //RCLCPP_INFO(this->get_logger(), "Message type info");
  message.type_info = dynmsg::c::get_type_info(InterfaceTypeName{"sensor_msgs", "JointState"});
  //RCLCPP_INFO(this->get_logger(), "Member count: %d ", message.type_info->member_count_);
  //RCLCPP_INFO(this->get_logger(), "Buffer");
  //message.data = reinterpret_cast<uint8_t *>(info.buffer);
  message.data = reinterpret_cast<uint8_t *>(msg);
  //RCLCPP_INFO(this->get_logger(), "Buffer length: %lu buffer capacity: %lu ", info.buffer_length, info.buffer_capacity);
  //RCLCPP_INFO(this->get_logger(), "Message to yaml..");
  auto yaml_msg = dynmsg::c::message_to_yaml(message);
  //RCLCPP_INFO(this->get_logger(), "Create emitter?");
  YAML::Emitter emitter;
  emitter <<  yaml_msg;

  std::cout << "YAML MSG: \n" << emitter.c_str() << std::endl;

}