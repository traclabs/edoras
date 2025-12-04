
#include <edoras_demos/arm_comm_default.h>
#include <tf2_kdl/tf2_kdl.hpp>

using std::placeholders::_1;

ArmCommDefault::ArmCommDefault() :
Node("arm_comm_default")
{
  this->declare_parameter("joint_state", std::string("/big_arm/joint_states"));
  this->declare_parameter("joint_command", std::string("/big_arm/joint_state_command"));
  this->declare_parameter("joint_state_throttled", std::string("/big_arm/joint_states_throttled"));  
  this->declare_parameter("target_pose", std::string("/big_arm/target_pose"));  
  this->declare_parameter("base_link", std::string("big_arm_link_1"));
  this->declare_parameter("tip_link", std::string("big_arm_link_8"));      

   robot_description_ = "robot_description";
   eps_ = 1e-5;
   max_time_ = 0.005;   
   solve_type_ = TRAC_IK::Speed;
   
   cmd_freq_ = 30.0;
   cmd_rate_ = 1.0/cmd_freq_; // 30 Hz

}


bool ArmCommDefault::initRobotComm()
{
  std::string js_topic;
  std::string jc_topic;
  std::string js_throttled;
  std::string target_topic;

  this->get_parameter("joint_state", js_topic);
  this->get_parameter("joint_state_throttled", js_throttled);
  this->get_parameter("joint_command", jc_topic);
  this->get_parameter("target_pose", target_topic);
  this->get_parameter("base_link", base_link_);
  this->get_parameter("tip_link", tip_link_);

  sub_cmd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = sub_cmd_cb_group_;

  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(js_topic, 10, std::bind(&ArmCommDefault::js_cb, this, _1));
  sub_target_ = this->create_subscription<geometry_msgs::msg::Pose>(target_topic, 10, std::bind(&ArmCommDefault::target_cb, this, _1), options);

  pub_jc_ = this->create_publisher<sensor_msgs::msg::JointState>(jc_topic, 10);
  pub_js_throttled_ = this->create_publisher<sensor_msgs::msg::JointState>(js_throttled, 10);  
   
  trac_ik_.reset( new TRAC_IK::TRAC_IK(shared_from_this(), base_link_, tip_link_, 
                   robot_description_, max_time_, eps_, solve_type_) );

  return true;
}


bool ArmCommDefault::initRest(const int &_tlm_ms)
{
   timer_tlm_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      
   timer_tlm_ = this->create_wall_timer(
      std::chrono::milliseconds(_tlm_ms),
      std::bind(&ArmCommDefault::send_telemetry, this), timer_tlm_cb_group_);

   return true;
}

void ArmCommDefault::send_telemetry()
{
   // Grab the latest data
   sensor_msgs::msg::JointState js;
   mux_.lock();
   js = joint_state_;
   mux_.unlock();   
   
   // Send it back
   if(js.name.size() == 0)
     return;

  pub_js_throttled_->publish(js);
}

// Extended example- Translation: [0.442, 4.248, 5.209] - Rotation: in Quaternion [0.851, 0.055, 0.039, 0.521]
// Zero example - - Translation: [0.903, 0.606, 4.755] - Rotation: in Quaternion [0.694, 0.324, 0.272, 0.583]
// Extended 2: - Translation: [-2.622, 1.686, 3.227] - Rotation: in Quaternion [0.860, -0.097, 0.340, 0.369]
/**
 * @function target_cb
 */
void ArmCommDefault::target_cb(const geometry_msgs::msg::Pose::SharedPtr _msg)
{
    RCLCPP_INFO(this->get_logger(), "Received pose command: %f %f %f -- %f %f %f %f", 
        _msg->position.x, _msg->position.y, _msg->position.z, 
        _msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w);
    
    // Do IK magic
    double jv = 5.0*M_PI/180.0;
    calculateMotion(*_msg, jv);
}

/**
 * @function calculateMotion
 */
bool ArmCommDefault::calculateMotion(const geometry_msgs::msg::Pose &_pose, const double &_jv)
{
  RCLCPP_INFO(this->get_logger(), "Calculating motion");
  // Fill IK data for problem
  KDL::JntArray qs, qg;
  KDL::Frame Tg;
  KDL::Twist bounds;
  
  Tg = KDL::Frame::Identity();
  tf2::fromMsg(_pose, Tg);
  bounds = KDL::Twist::Zero();
  
  sensor_msgs::msg::JointState js;
  mux_.lock(); js = joint_state_; mux_.unlock();  

RCLCPP_INFO_STREAM(this->get_logger(), "Calculate motion, js start: " << sensor_msgs::msg::to_yaml(js));


  qs.data.resize(js.position.size());
  for(size_t i = 0; i < js.position.size(); ++i)
    qs(i) = js.position[i];

  int res = trac_ik_->CartToJnt(qs, Tg, qg, bounds);
  if(res < 0)
  {
    RCLCPP_INFO(this->get_logger(), "IK result was: %d returning", res);
    return false;
  }  
  // If a solution is found, move the robot
  rclcpp::Rate r(cmd_freq_);

  Eigen::VectorXd qi, dq, dqi;
  dq = (qg.data - qs.data);

  RCLCPP_INFO(this->get_logger(), "Executing motion. Res: %d ", res);
  size_t counts = ( dq.norm()/_jv ) * cmd_freq_;
  dqi = dq/(double)counts;
    
  sensor_msgs::msg::JointState ji;
  
  ji = js;
  for(size_t i = 0; i < counts; ++i)
  {
     qi = qs.data + dqi*((double)i);
     for(size_t k = 0; k < ji.position.size(); ++k)
       ji.position[k] = qi(k);
        
     pub_jc_->publish(ji);
     r.sleep();
  }
  
  return true;
}


/**
 * @function js_cb
 */
void ArmCommDefault::js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  mux_.lock();
 joint_state_ = *_msg;
 mux_.unlock();
}

