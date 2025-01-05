/**
 * @file fake_base_cmd.cpp
 */
#include <edoras_demos/fake_base_cmd.h>

using std::placeholders::_1, std::placeholders::_2;

void setTransform(double _x, double _y, double _z,
                  double _roll, double _pitch, double _yaw,
                  tf2::Transform &_tf)
{
  tf2::Quaternion q;
  _tf.setOrigin(tf2::Vector3(_x, _y, _z));
  q.setRPY(_roll, _pitch, _yaw);
  _tf.setRotation(q);
}

std::string clean_name(const std::string &_name)
{
  std::string name = _name;
  if(name.size() == 0)
    return name;

  if (name[0] == '/')
    name = name.substr(1);

  return name;
}

/**
 * @function FakeBaseCmd
 * @brief Constructor
 */
FakeBaseCmd::FakeBaseCmd() :
Node("fake_base_cmd") {

  this->declare_parameter("planning_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("base_link_frame", rclcpp::PARAMETER_STRING);
  this->declare_parameter("cmd_vel", rclcpp::PARAMETER_STRING);
  this->declare_parameter("x", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("y", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("z", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("roll", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("pitch", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("yaw", rclcpp::PARAMETER_DOUBLE);
}

/**
 * @function vel_cmd_cb
 * @brief Twist callback
 */
void FakeBaseCmd::vel_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  vel_mux_.lock();
  vel_msg_ = *_msg;
  vel_msg_stamp_ = this->now();
  vel_mux_.unlock();
}

/**
 * @function init
 * @brief initialize
 */
bool FakeBaseCmd::init()
{
  // Init time-related variables
  tf_buffer_  = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Start with reference_frame and frame_id in same location
  x_ = 0;
  y_ = 0;
  z_ = 0.0;
  ra_ = 0;
  pa_ = 0;
  ya_ = 0;
 
  this->get_parameter("base_link_frame", base_link_frame_);
  this->get_parameter("planning_frame", fixed_frame_);
 
  // clean loaded parameters
  base_link_frame_ = clean_name(base_link_frame_);
  fixed_frame_ = clean_name(fixed_frame_);
 
  this->get_parameter("cmd_vel", cmd_vel_topic_);
  this->get_parameter("x", x_);
  this->get_parameter("y", y_);
  this->get_parameter("z", z_);
  this->get_parameter("roll", ra_);
  this->get_parameter("pitch", pa_);
  this->get_parameter("yaw", ya_);
 
  // Summary output
  RCLCPP_INFO(this->get_logger(),
      "fake_base_cmd: \n\t * Base link frame: %s \n\t * Fixed frame: %s \n\t * cmd_vel: %s "
      "* x: %f y: %f z: %f roll: %f pitch: %f yaw: %f",
      base_link_frame_.c_str(), fixed_frame_.c_str(), cmd_vel_topic_.c_str(),
       x_, y_, z_, ra_, pa_, ya_);

  // Initialize tf_base
  tf_base_.setIdentity();
 
  // Start by setting robot in, well, starting pose 
  // (either zero by default or whatever the user specified as argument)
  setTransform(x_, y_, z_, ra_, pa_, ya_, tf_robot_reference_);

/*
  tf_buffer_->canTransform(base_link_frame_, robot_reference_frame_, rclcpp::Time(0), rclcpp::Duration(std::chrono::seconds(3)));
  auto base_link_to_robot_reference = tf_buffer_->lookupTransform(robot_reference_frame_, base_link_frame_,
  tf2::TimePointZero);
  tf2::convert(base_link_to_robot_reference.transform, base_to_robot_ref_tf_);
  // handle any offset between the base and robot frame
  tf_base_ = tf_robot_reference_ * base_to_robot_ref_tf_;
*/
  tf_base_ = tf_robot_reference_;
  geometry_msgs::msg::TransformStamped fixed_to_base;
  tf2::convert(tf_base_, fixed_to_base.transform);
  fixed_to_base.header.frame_id = fixed_frame_;
  fixed_to_base.child_frame_id = base_link_frame_;
  fixed_to_base.header.stamp = this->now();
 
  // To start us off:
  br_->sendTransform(fixed_to_base);
  RCLCPP_INFO(this->get_logger(), "Publishing tf between %s and %s", fixed_frame_.c_str(), base_link_frame_.c_str());
  rclcpp::spin_some(shared_from_this());
  rclcpp::sleep_for(std::chrono::milliseconds(100));
 RCLCPP_INFO(this->get_logger(), "DEBUG 8");
  // 1. Connect to publisher cmd_vel
  vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1,
                  std::bind(&FakeBaseCmd::vel_cmd_cb, this, _1));

  return true;
}

/**
 * @function spin
 */
void FakeBaseCmd::spin()
{
  int hertz = 100;
  double dt = 1.0 / (double)(hertz);  // 10Hz
  rclcpp::Rate r(std::chrono::milliseconds(1000/hertz));
  double thresh_vel_z = 0.001;

  // Grab latest
   geometry_msgs::msg::Twist vel_msg;
   rclcpp::Time vel_msg_stamp;

  rclcpp::Time current_time;
  while (rclcpp::ok())
  {
    // Get latest vel, if available
    vel_mux_.lock();
    vel_msg = vel_msg_;
    vel_msg_stamp = vel_msg_stamp_;
    vel_mux_.unlock();

    current_time = this->now();
    // If msg is too old keep going
    if ((current_time.seconds() - rclcpp::Time(vel_msg_stamp).seconds()) < 0.2)
    {
      x_ += vel_msg.linear.x * cos(ya_) * dt;
      y_ += vel_msg.linear.x * sin(ya_) * dt;
      if (fabs(vel_msg.linear.z) > thresh_vel_z)
      {
        z_ += vel_msg.linear.z * dt;
      }
      ya_ += vel_msg.angular.z * dt;
    }

    setTransform(x_, y_, z_, ra_, pa_, ya_, tf_robot_reference_);
    tf_base_ = tf_robot_reference_; // * base_to_robot_ref_tf_;

    geometry_msgs::msg::TransformStamped fixed_to_base;
    tf2::convert(tf_base_, fixed_to_base.transform);
    fixed_to_base.header.frame_id = fixed_frame_;
    fixed_to_base.child_frame_id = base_link_frame_;
    fixed_to_base.header.stamp = this->now();
    br_->sendTransform(fixed_to_base);

    // Sleep period
    r.sleep();
    rclcpp::spin_some(shared_from_this());
  }

}


/**
 * @function main
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto fake_base_cmd = std::make_shared<FakeBaseCmd>();

  RCLCPP_WARN(fake_base_cmd->get_logger(), "Starting fake_base_cmd node");
  if(!fake_base_cmd->init())
  {
    RCLCPP_ERROR(fake_base_cmd->get_logger(), "Error initializing fake_base_cmd");
    return 1;
  }

  // 2. Keep listening and once cmd_vel is gotten, integrate and move the base transform
  fake_base_cmd->spin();

  return 0;
}
