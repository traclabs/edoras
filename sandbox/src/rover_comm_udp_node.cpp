#include <edoras_demos/rover_comm_udp.h>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoverCommUdp>();

  std::string fixed_frame = "world";
  std::string base_link_frame = "base_footprint";
  std::string cmd_vel_topic = "cmd_vel"; 
  int cfs_port = 8080;
  int robot_port = 8585;
  int tlm_ms = 2000;
  int cmd_ms = 100;
  
  RCLCPP_INFO(node->get_logger(), "Init robot comm");
  node->initRobotComm(fixed_frame, base_link_frame, cmd_vel_topic);
  
  RCLCPP_INFO(node->get_logger(), "Init udp comm");
  if(!node->initUdpComm(cfs_port, robot_port))
  {
    RCLCPP_ERROR(node->get_logger(), "Error initializing communication");
    return 1;
  }
  
  RCLCPP_INFO(node->get_logger(), "Init rest");
  node->initRest(tlm_ms, cmd_ms);
    RCLCPP_INFO(node->get_logger(), "Spin...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
