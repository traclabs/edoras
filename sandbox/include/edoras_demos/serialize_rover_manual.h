#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

class SerializeRoverManual
{
 public:
 
 SerializeRoverManual();
 bool initializeComm( const int &_own_port, const int &_other_port, std::string &_error_msg);
 bool sendMessage( geometry_msgs::msg::Pose* _pm );
 bool receiveMessage(geometry_msgs::msg::Twist &_tm);
 
 protected:

  size_t serialize(geometry_msgs::msg::Pose* _js, unsigned char** buf);
  geometry_msgs::msg::Twist deserialize(const uint8_t* buf, const size_t bufSize, size_t start_offset);

  int sockfd_;
  char buffer_[1024];
  struct sockaddr_in own_address_;
  struct sockaddr_in other_address_;
 
 
};
