#pragma once 

class SbnReceiver
{
 public:
   SbnReceiver(const std::string _udp_ip, const int &_udp_receive_port);
   bool addPeer(const std::string &_peer_udp_ip, const int &_peer_port, 
                const int &_peer_spacecraft_id, const int &_peer_processor_id);
                
 protected:
                 
};
