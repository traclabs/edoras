#include <edoras_demos/arm_comm_udp.h>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCommUdp>();
  
  std::string js_topic = "/big_arm/joint_states";
  std::string jc_topic = "/big_arm/joint_state_command";
  int cfs_port = 8080;
  int robot_port = 8585;
  int tlm_ms = 2000;
  int cmd_ms = 100;
  
  node->initRobotComm(js_topic, jc_topic);
  
  if(!node->initUdpComm(cfs_port, robot_port))
  {
    RCLCPP_ERROR(node->get_logger(), "Error initializing communication");
    return 1;
  }
  
  node->initRest(tlm_ms, cmd_ms);  

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
