#include <edoras_demos/arm_comm_udp.h>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCommUdp>();
  
  std::string js_topic = "/big_arm/joint_states";
  
  RCLCPP_INFO(node->get_logger(), "Init robot comm");
  node->initRobotComm(js_topic);
  
  RCLCPP_INFO(node->get_logger(), "Init udp comm");
  if(!node->initUdpComm())
  {
    RCLCPP_ERROR(node->get_logger(), "Error initializing communication");
    return 1;
  }
  
  RCLCPP_INFO(node->get_logger(), "Init rest");
  node->initRest();
    RCLCPP_INFO(node->get_logger(), "Spin...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
