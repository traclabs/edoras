#include <yaml-cpp/yaml.h>

#include <string>

#include "dynmsg/message_reading.hpp"
#include "dynmsg/msg_parser.hpp"
#include "dynmsg/typesupport.hpp"
#include "dynmsg/yaml_utils.hpp"

#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/joint_state.h"

int main()
{
  // Start with a ROS 2 message, like a std_msgs/Header
  sensor_msgs__msg__JointState * msg = sensor_msgs__msg__JointState__create();
  rosidl_runtime_c__double__Sequence__init(&msg->position, 3);
  msg->position.data[0] = 0.2;
  msg->position.data[1] = 0.3;
  msg->position.data[2] = 0.4;

/*  msg->stamp.sec = 4;
  msg->stamp.nanosec = 20u;
  rosidl_runtime_c__String__assign(&msg->frame_id, "my_frame");
*/
  // Convert it into a YAML representation
  RosMessage_C ros_msg;
  // Note: type support info could be retrieved through other means, see dynmsg::cpp::*
  InterfaceTypeName interface{"sensor_msgs", "JointState"};
  ros_msg.type_info = dynmsg::c::get_type_info(interface);
  ros_msg.data = reinterpret_cast<uint8_t *>(msg);
  YAML::Node yaml_msg = dynmsg::c::message_to_yaml(ros_msg);

  // Convert the YAML representation to a string
  const std::string yaml_string = dynmsg::yaml_to_string(yaml_msg);
  printf("%s\n", yaml_string.c_str());

  printf("\n");

  // Convert YAML string back to a ROS 2 message
  RosMessage_C ros_msg_from_yaml = dynmsg::c::yaml_to_rosmsg(interface, yaml_string);
  auto msg_from_yaml = reinterpret_cast<sensor_msgs__msg__JointState *>(ros_msg_from_yaml.data);
  // Prints:
  //   my_frame
  //   4 s, 20 ns
//  printf("%s\n", msg_from_yaml->frame_id.data);
  printf("pos[0]: %f , pos[1]: %f \n", msg_from_yaml->position.data[0], msg_from_yaml->position.data[1]);

  return 0;
}
