#include <edoras_demos/arm_comm_default.h>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCommDefault>();
  
  int tlm_ms = 1000;
  
  node->initRobotComm();
    
  node->initRest(tlm_ms);  

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
