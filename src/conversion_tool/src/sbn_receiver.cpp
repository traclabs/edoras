/**
 * @file sbn_receiver.cpp
 */
#include <conversion_tool/sbn_receiver.h>


SbnReceiver::SbnReceiver(const std::string _udp_ip, const int &_udp_receive_port)
{
   udp_ip_ = _udp_ip;
   udp_receive_port_ = _udp_receive_port;
   timer_period_ = 0.1;
   recv_buff_size_ = 4096;
   
   // Create socket
}


bool SbnReceiver::addPeer(const std::string &_peer_udp_ip, const int &_peer_port, 
                          const int &_peer_spacecraft_id, const int &_peer_processor_id)
{

   return true;
}                          
                        
