#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>

#include <sensor_msgs/msg/joint_state.hpp>


class SerializeManual
{
 public:
 
 SerializeManual();
 bool initializeComm( const int &_own_port, const int &_other_port, std::string &_error_msg);
 bool sendMessage( sensor_msgs::msg::JointState* _js );
 bool receiveMessage(sensor_msgs::msg::JointState &_js);
 
 protected:

  size_t serialize(sensor_msgs::msg::JointState* _js, unsigned char** buf);
  sensor_msgs::msg::JointState deserialize(const unsigned char* buf, const size_t bufSize, size_t start_offset);

  int sockfd_;
  char buffer_[1024];
  struct sockaddr_in own_address_;
  struct sockaddr_in other_address_;
 
 
};