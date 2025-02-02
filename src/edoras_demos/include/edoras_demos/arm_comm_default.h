/**
 * @file arm_comm_default.h
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <trac_ik/trac_ik.hpp>

class ArmCommDefault : public rclcpp::Node {

public:

  ArmCommDefault();
  bool initRobotComm();
  bool initRest(const int &_tlm_ms);
  
protected:

  void send_telemetry();
  
  void js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg);
  void target_cb(const geometry_msgs::msg::Pose::SharedPtr _msg);
  bool calculateMotion(const geometry_msgs::msg::Pose &_pose, const double &_jv);
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_target_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_jc_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_throttled_;
    
  rclcpp::TimerBase::SharedPtr timer_tlm_;
  rclcpp::CallbackGroup::SharedPtr timer_tlm_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cmd_cb_group_;
      
  std::mutex mux_;
      
  sensor_msgs::msg::JointState joint_state_;
  std::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_;
  std::shared_ptr<rclcpp::Node> trac_ik_sub_node_;
  double cmd_freq_;
  double cmd_rate_;

  std::string base_link_;
  std::string tip_link_;
  std::string robot_description_;
  double max_time_; 
  double eps_;
  TRAC_IK::SolveType solve_type_;


};

