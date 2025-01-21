/**
 * @file basic_communication.h
 */
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <string>
#include <cstring> // memset
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <conversion_tool/sbn_peer.h>

/**
 * @class PeerCommunication
 */
class PeerCommunication
{
 public:
 
 PeerCommunication(std::shared_ptr<rclcpp::Node> _node);
 bool initialize(const int &_own_port, 
		 const std::string &_telemetry_ip,
		 const uint32_t & _spacecraft_id, 
	         const uint32_t & _processor_id,
		 std::string &_error_msg);

 bool addPeer(const std::string &_peer_ip,
              const int &_peer_port,
	      const uint32_t & _peer_spacecraft_id, 
	      const uint32_t & peer_processor_id);
	      
 void sendAllSubscriptionMsg(const uint16_t &_mid);
 void sendAllUnsubscriptionMsg(const uint16_t &_mid);
 
  
 /*
 bool sendCmdPacket(const uint16_t &_mid, const uint8_t &_code, 
                    const uint16_t &_seq, unsigned char** _data_buffer, 
                    const size_t &_data_size); */
 bool receiveTlmPacket(uint16_t &_mid, uint8_t** _buffer);

 protected:
 
 SbnPeer* getPeer(const uint32_t &_peer_spacecraft_id, 
                  const uint32_t &_peer_processor_id);
 
 bool findPeer(const uint32_t &_peer_spacecraft_id, 
               const uint32_t &_peer_processor_id);

 bool updatePeer(SbnPeer* _peer);

 int handleSbnMsg(uint8_t* _buf, const ssize_t &_buf_size,
                  uint16_t &_mid, uint8_t** _buffer_tlm);
 

  size_t readSbnHeader(uint8_t* _buf, 
                       const uint8_t &_buf_size,
                       uint16_t &msg_size,
                       uint8_t &_msg_type,
                       uint32_t &_spacecraft_id, 
                       uint32_t &_processor_id);

  void processSbnSubscriptionMsg(uint8_t *_buf, SbnPeer* _peer);
  void processSbnUnsubscriptionMsg(uint8_t *_buf, SbnPeer* _peer);
  void processSbnProtocolMsg(uint8_t *_buf);
  void parseSbnSubMsg(uint8_t* _buf, std::vector<SubscriptionData> &_subscriptions);
  bool processCfeTlmMessage(uint8_t *_buf, SbnPeer* _peer, const ssize_t &_buf_size, 
                            uint16_t &_mid, uint8_t** _buffer_tlm);
//  size_t createCmdPacket(const uint16_t &_mid, const uint8_t &_code, const uint16_t &_seq, 
//                         const size_t &_data_size, unsigned char** _data_buffer, unsigned char** _cmd_packet);

  private:
  
  bool sendSubscriptionMsg(SbnPeer* _peer, const uint16_t &_mid);
  bool sendSubscriptionMsg(SbnPeer* _peer, const std::vector<uint16_t> &_mid); 
  bool sendProtocolMsg(SbnPeer* _peer);
  bool sendHeartbeat(SbnPeer* _peer);
  
  bool send(uint8_t* _packet, 
            const size_t &_packet_size, 
            SbnPeer* _peer);

  // Create helpers
  uint8_t* createSubscriptionMsg(size_t &_msg_size,
                             const uint8_t &_msg_type,
                             const std::vector<uint16_t> &_mids, 
                             const uint8_t &_sbn_sub_qos_priority,
                             const uint8_t &_sbn_sub_qos_reliability);
 size_t writeSbnHeader(uint8_t** _msg, 
                      const uint16_t &_msg_size, 
                      const uint8_t &_msg_type);


  protected:
  int sock_fd_;
  char buffer_[1024];
  
  int udp_receive_port_; 
  std::string udp_ip_;
  uint32_t spacecraft_id_;
  uint32_t processor_id_;
  
  // SBN
  static const char* rev_id_string_;
  static const size_t rev_id_size_;
  static const size_t EDORAS_SBN_HDR_SIZE;
  static const size_t EDORAS_TLM_HDR_SIZE;
  
  struct sockaddr_in own_address_;
  std::vector<SbnPeer> peers_; 
  
  std::shared_ptr<rclcpp::Node> node_;
};
