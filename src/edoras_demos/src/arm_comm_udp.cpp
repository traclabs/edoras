
#include <edoras_demos/arm_comm_udp.h>

using std::placeholders::_1;

ArmCommUdp::ArmCommUdp() :
Node("arm_comm_udp")
{   
   base_link_ = "big_arm_link_0";
   tip_link_ = "big_arm_link_8";
   robot_description_ = "robot_description";
   eps_ = 1e-5;
   max_time_ = 0.005;   
   solve_type_ = TRAC_IK::Speed;
   
   trac_ik_.reset( new TRAC_IK::TRAC_IK(this->create_sub_node("trac_ik"), base_link_, tip_link_, 
                   robot_description_, max_time_, eps_, solve_type_) );
}


bool ArmCommUdp::initRobotComm(const std::string &_js_topic)
{
  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(_js_topic, 10, std::bind(&ArmCommUdp::js_cb, this, _1));
  return true;
}


bool ArmCommUdp::initUdpComm(const int &_cfs_port, 
                             const int &_robot_port)
{
   cfs_port_ = _cfs_port;
   robot_port_ = _robot_port;
   
   std::string error_msg;
   
   return sm_.initializeComm(robot_port_, cfs_port_, error_msg);
}

bool ArmCommUdp::initRest(const int &_tlm_ms, const int &_cmd_ms)
{
   timer_tlm_ = this->create_wall_timer(
      std::chrono::milliseconds(_tlm_ms),
      std::bind(&ArmCommUdp::send_telemetry, this));

   timer_cmd_ = this->create_wall_timer(
      std::chrono::milliseconds(_cmd_ms),
      std::bind(&ArmCommUdp::rcv_command, this));

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

   RCLCPP_INFO(this->get_logger(), "Sending telemetry: sensor_msgs::JointState serialized as 7 joint values");
   RCLCPP_INFO(this->get_logger(), " Joints: %f %f %f %f %f %f %f", js.position[0], js.position[1], js.position[2], js.position[3], js.position[4], js.position[5], js.position[6] );
       
   if(!sm_.sendMessage(&js))
     RCLCPP_ERROR(this->get_logger(), "Error sending message");
}

/**
 * @function rcv_command
 */
void ArmCommUdp::rcv_command()
{
  geometry_msgs::msg::Pose cmd;

  if(sm_.receiveMessage(cmd))
  {
    RCLCPP_INFO(this->get_logger(), "Received pose command: %f %f %f -- %f %f %f %f", cmd.position.x, cmd.position.y, cmd.position.z, cmd.orientation.x, cmd.orientation.y, cmd.orientation.z, cmd.orientation.w);
    
    // Do IK magic
    //pub_twist_->publish(cmd);
  }  
}

/**
 * @function js_cb
 */
void ArmCommUdp::js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  mux_.lock();
 joint_state_ = *_msg;
 mux_.unlock();
}

