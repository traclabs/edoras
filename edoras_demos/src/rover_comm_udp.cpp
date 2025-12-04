
#include <edoras_demos/rover_comm_udp.h>

using std::placeholders::_1;

RoverCommUdp::RoverCommUdp() :
Node("rover_comm_udp")
{
  this->declare_parameter("fixed_frame", std::string("world"));
  this->declare_parameter("base_link_frame", std::string("base_footprint"));
  this->declare_parameter("cmd_vel_topic", std::string("simulate_control_vel"));
    
  this->declare_parameter("cfs_port", 8080);
  this->declare_parameter("robot_port", 8585);  
  this->declare_parameter("cfs_ip", std::string("127.0.0.1"));
  this->declare_parameter("robot_ip", std::string("127.0.0.1"));    
}

/** 
 * @function initRobotComm
 * @brief ROS2 stuff to talk to/control the robot
 */
bool RoverCommUdp::initRobotComm()
{ 
  std::string fixed_frame;
  std::string base_link_frame;
  std::string cmd_vel_topic;

  this->get_parameter("fixed_frame", fixed_frame);  
  this->get_parameter("base_link_frame", base_link_frame);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic);  


  fixed_frame_ = fixed_frame;
  base_link_frame_ = base_link_frame;
  cmd_vel_topic_ = cmd_vel_topic;
  
  pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  
  // Will check TF periodically to get pose to send back as telemetry
  tf_buffer_  = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  //br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  
  return true;
}


bool RoverCommUdp::initUdpComm()
{
  int cfs_port;
  int robot_port;
  std::string cfs_ip;
  std::string robot_ip;
    
  this->get_parameter("cfs_port", cfs_port);  
  this->get_parameter("robot_port", robot_port);
  this->get_parameter("cfs_ip", cfs_ip);  
  this->get_parameter("robot_ip", robot_ip);      

  RCLCPP_INFO(this->get_logger(), "** initUdpComm: cfs port: %d cfs ip: %s robot port: %d robot ip: %s", 
              cfs_port, cfs_ip.c_str(), robot_port, robot_ip.c_str());

   cfs_port_ = cfs_port;
   robot_port_ = robot_port;
   cfs_ip_ = cfs_ip;
   robot_ip_ = robot_ip;
   
   std::string error_msg;
   
   return sm_.initializeComm(robot_port_, cfs_port_, robot_ip_, cfs_ip_, error_msg);
}

bool RoverCommUdp::initRest(const int &_tlm_ms, const int &_cmd_ms)
{
   timer_tlm_ = this->create_wall_timer(
      std::chrono::milliseconds(_tlm_ms),
      std::bind(&RoverCommUdp::send_telemetry, this));

   timer_cmd_ = this->create_wall_timer(
      std::chrono::milliseconds(_cmd_ms),
      std::bind(&RoverCommUdp::rcv_command, this));

   return true;
}

/**
 * @function send_telemetry
 */
void RoverCommUdp::send_telemetry()
{
  // Check robot pose
  auto time_now = this->now();
  tf_buffer_->canTransform(base_link_frame_, fixed_frame_, time_now, rclcpp::Duration(std::chrono::seconds(3)));
  auto robot_tf = tf_buffer_->lookupTransform(fixed_frame_, base_link_frame_, tf2::TimePointZero);

  robot_pose_.pose.orientation = robot_tf.transform.rotation;
  robot_pose_.pose.position.x = robot_tf.transform.translation.x;  
  robot_pose_.pose.position.y = robot_tf.transform.translation.y;  
  robot_pose_.pose.position.z = robot_tf.transform.translation.z;  
  robot_pose_.header.stamp = time_now;     
       
  auto ps = robot_pose_;
       
   // Send it back
   if(ps.pose.orientation.w == 0)
     return;
     
   RCLCPP_INFO(this->get_logger(), "Sending telemetry: geometry_msgs::Pose serialized as 7 doubles");
   RCLCPP_INFO(this->get_logger(), " pos: %f %f %f orient: %f %f %f %f. Time: %d %d", 
               ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, 
               ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w,
               ps.header.stamp.sec, ps.header.stamp.nanosec);
       
   if(!sm_.sendMessage(&ps))
     RCLCPP_ERROR(this->get_logger(), "Error sending message");
}

/**
 * @function rcv_command
 */
void RoverCommUdp::rcv_command()
{
  geometry_msgs::msg::Twist cmd;

  if(sm_.receiveMessage(cmd))
  {
    RCLCPP_INFO(this->get_logger(), "Received twist command: %f %f %f -- %f %f %f", cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z);
    pub_twist_->publish(cmd);
  }  
}
