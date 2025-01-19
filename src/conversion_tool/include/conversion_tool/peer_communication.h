/**
 * @file basic_communication.h
 */
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <cstring> // memset
#include <vector>
#include <rclcpp/rclcpp.hpp>

class SbnPeer
{
  public:
  
   SbnPeer(const std::string &_peer_ip,
           const int &_peer_port,
	   const int & _peer_spacecraft_id, 
	   const int & _peer_processor_id,
	   const int &_spacecraft_id, 
	   const int &_processor_id);

   std::string udp_ip_;
   int udp_port_;

   // Remote
   int peer_spacecraft_id_;
   int peer_processor_id_;

   // Local
   int spacecraft_id_;
   int processor_id_;
   
   bool connected_;
   rclcpp::Time last_heartbeat_rx_;
   rclcpp::Time last_tx_;
   double time_period_;
   
   
};

/**
 * @class PeerCommunication
 */
class PeerCommunication
{
 public:
 
 PeerCommunication();
 bool initialize(const int &_own_port, 
		 const std::string &_telemetry_ip,
		 const int & _spacecraft_id, 
	         const int & _processor_id,
		 std::string &_error_msg);

 bool addPeer(const std::string &_peer_ip,
              const int &_peer_port,
	      const int & _peer_spacecraft_id, 
	      const int & peer_processor_id);
	      
 void sendAllSubscriptionMsg(const uint16_t &_mid);
 void sendAllUnsubscriptionMsg(const uint16_t &_mid);
 /*
 bool sendCmdPacket(const uint16_t &_mid, const uint8_t &_code, 
                    const uint16_t &_seq, unsigned char** _data_buffer, 
                    const size_t &_data_size);
 bool receiveTlmPacket(uint16_t &_mid, uint8_t** _buffer, 
                       std::vector<uint8_t> &_header_debug, 
                       size_t &_buffer_size);
 */
 protected:
 
 bool find(const int &_peer_spacecraft_id, 
          const int &_peer_processor_id);

//  size_t createCmdPacket(const uint16_t &_mid, const uint8_t &_code, const uint16_t &_seq, 
//                         const size_t &_data_size, unsigned char** _data_buffer, unsigned char** _cmd_packet);

  int sock_fd_;
  char buffer_[1024];
  
  int udp_receive_port_; 
  std::string udp_ip_;
  int spacecraft_id_;
  int processor_id_;
  
  // SBN
  char rev_id_string_[48];
  
  struct sockaddr_in own_address_;
  std::vector<SbnPeer> peers_; 
};
