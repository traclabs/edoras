#include <edoras_demos/rover_comm_udp.h>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoverCommUdp>();

  int tlm_ms = 2000;
  int cmd_ms = 100;
      
  node->initRobotComm();
  
  if(!node->initUdpComm())
    return 1;
  
  node->initRest(tlm_ms, cmd_ms);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
