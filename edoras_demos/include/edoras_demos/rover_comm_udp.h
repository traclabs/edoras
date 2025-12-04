
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <edoras_demos/serialize_rover_manual.h>

class RoverCommUdp : public rclcpp::Node {

public:

  RoverCommUdp();
  bool initRobotComm();
  bool initUdpComm();
  bool initRest(const int &_tlm_ms, const int &_cmd_ms);
  
protected:

  void send_telemetry();
  void rcv_command();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::TimerBase::SharedPtr timer_tlm_;
  rclcpp::TimerBase::SharedPtr timer_cmd_;
  std::mutex mux_;
  
  SerializeRoverManual sm_;
  int cfs_port_;
  int robot_port_;
  std::string cfs_ip_;
  std::string robot_ip_;
  
  std::string fixed_frame_;
  std::string base_link_frame_;
  
  geometry_msgs::msg::PoseStamped robot_pose_;
  std::string cmd_vel_topic_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  
};

