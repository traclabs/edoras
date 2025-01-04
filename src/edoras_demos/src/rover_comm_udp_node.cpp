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
  
  node->initRobotComm(fixed_frame, base_link_frame, cmd_vel_topic);
  
  if(!node->initUdpComm(cfs_port, robot_port))
    return 1;
  
  node->initRest(tlm_ms, cmd_ms);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
