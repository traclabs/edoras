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

   std::string udp_ip;
   int udp_port;
   struct sockaddr_in address;
   
   // Remote
   uint32_t peer_spacecraft_id;
   uint32_t peer_processor_id;

   // Local
   uint32_t spacecraft_id;
   uint32_t processor_id;
   
   bool connected;
   rclcpp::Time last_heartbeat_rx;
   rclcpp::Time last_tx;
   double time_period;
   std::vector<uint16_t> ros_subscriptions;   

   bool hasRosSubscription(const uint16_t &_mid);
   void addRosSubscription(const uint16_t &_mid);
};


/**
 * @class PeerCommunication
 */
class PeerCommunication
{
 public:
 
 PeerCommunication(std::shared_ptr<rclcpp::Node> _node);
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
 
 void sendSubscriptionMsg(SbnPeer &_peer, const uint16_t &_mid); 
 
 
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

 uint8_t*  createSubscriptionMsg(size_t &_msg_size,
                            const uint8_t &_msg_type,
                            const uint16_t &_sbn_count, 
                            const uint16_t &_mid, 
                            const uint8_t &_sbn_sub_qos_priority,
                            const uint8_t &_sbn_sub_qos_reliability,
                            SbnPeer &_peer);

 size_t writeSbnHeader(uint8_t** _msg, 
                      const uint16_t &_msg_size, 
                      const uint8_t &_msg_type);


//  size_t createCmdPacket(const uint16_t &_mid, const uint8_t &_code, const uint16_t &_seq, 
//                         const size_t &_data_size, unsigned char** _data_buffer, unsigned char** _cmd_packet);

  bool send(uint8_t* _packet, 
            const size_t &_packet_size, 
            SbnPeer &_peer);

  int sock_fd_;
  char buffer_[1024];
  
  int udp_receive_port_; 
  std::string udp_ip_;
  int spacecraft_id_;
  int processor_id_;
  
  // SBN
  static const char* rev_id_string_;
  static const size_t rev_id_size_;
  
  struct sockaddr_in own_address_;
  std::vector<SbnPeer> peers_; 
  
  std::shared_ptr<rclcpp::Node> node_;
};
