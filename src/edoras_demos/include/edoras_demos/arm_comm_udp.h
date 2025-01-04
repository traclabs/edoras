/**
 * @file arm_comm_udp.h
 */
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <edoras_demos/serialize_arm_manual.h>

class ArmCommUdp : public rclcpp::Node {

public:

  ArmCommUdp();
  bool initRobotComm(const std::string &_joint_state);
  bool initUdpComm(const int &_cfs_port, const int &_robot_port);
  bool initRest(const int &_tlm_ms, const int &_cmd_ms);
  
protected:

  void send_telemetry();
  void rcv_command();
  
  void js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;

  rclcpp::TimerBase::SharedPtr timer_tlm_;
  rclcpp::TimerBase::SharedPtr timer_cmd_; 
  std::mutex mux_;
  
  SerializeArmManual sm_;
  int cfs_port_;
  int robot_port_;
  
  sensor_msgs::msg::JointState joint_state_;
};

