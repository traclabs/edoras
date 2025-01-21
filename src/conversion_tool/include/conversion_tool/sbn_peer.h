/**
 * @file basic_communication.h
 */
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

struct SubscriptionData {
  uint16_t mid;
  uint8_t qos_priority;
  uint8_t qos_reliability;
  
  SubscriptionData(const uint16_t &_mid, 
                   const uint8_t &_qos_priority, 
                   const uint8_t &_qos_reliability);
};

/**
 * @class SbnPeer
 */
class SbnPeer
{
  public:
  
   SbnPeer(const std::string &_peer_ip,
           const int &_peer_port,
	   const uint32_t & _peer_spacecraft_id, 
	   const uint32_t & _peer_processor_id,
	   const uint32_t &_spacecraft_id, 
	   const uint32_t &_processor_id);

   bool setConnected( const rclcpp::Time &_now);
   
   bool hasRosSubscription(const uint16_t &_mid);
   void addRosSubscription(const uint16_t &_mid);
   
   bool hasSubscription(const uint16_t &_mid);   
   size_t addSubscriptions(const std::vector<SubscriptionData> &_subscriptions);
   size_t deleteSubscriptions(const std::vector<SubscriptionData> &_subscriptions);
   
   // Variables
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
   std::vector<SubscriptionData> subscriptions;

};



