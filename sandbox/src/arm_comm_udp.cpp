
#include <edoras_demos/arm_comm_udp.h>

using std::placeholders::_1;

ArmCommUdp::ArmCommUdp() :
Node("arm_comm_udp")
{

}


bool ArmCommUdp::initRobotComm(const std::string &_js_topic)
{
  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(_js_topic, 10, std::bind(&ArmCommUdp::js_cb, this, _1));
  return true;
}


bool ArmCommUdp::initUdpComm()
{
   int own_port = 8585; // robot port
   int cfs_port = 8080;
   std::string error_msg;
   
   if(!sm_.initializeComm(own_port, cfs_port, error_msg))
     return false;
     
   return true;
}

bool ArmCommUdp::initRest()
{
   timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000),
      std::bind(&ArmCommUdp::send_telemetry, this));


   return true;
}

void ArmCommUdp::send_telemetry()
{
   // Grab the latest data
   sensor_msgs::msg::JointState js;
   mux_.lock();
   js = joint_state_;
   mux_.unlock();   
   
   // Send it back
   if(js.name.size() == 0)
     return;
   RCLCPP_INFO(this->get_logger(), "Sending telemetry!");
   RCLCPP_INFO(this->get_logger(), " Joints: %f %f %f %f %f %f %f", js.position[0], js.position[1], js.position[2], js.position[3], js.position[4], js.position[5], js.position[6] );    
   if(!sm_.sendMessage(&js))
     RCLCPP_ERROR(this->get_logger(), "Error sending message");
}

void ArmCommUdp::js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  mux_.lock();
 joint_state_ = *_msg;
 mux_.unlock();
}

