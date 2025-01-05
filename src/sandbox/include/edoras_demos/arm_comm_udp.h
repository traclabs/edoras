
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <edoras_demos/serialize_manual.h>

class ArmCommUdp : public rclcpp::Node {

public:

  ArmCommUdp();
  bool initRobotComm(const std::string &_js_topic);
  bool initUdpComm();
  bool initRest();
  
protected:

  void send_telemetry();
  void js_cb(const sensor_msgs::msg::JointState::SharedPtr _msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mux_;
  
  SerializeManual sm_;
  
  sensor_msgs::msg::JointState joint_state_;
};

