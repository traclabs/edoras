
#include <conversion_tool/sbn_peer.h>
#include <arpa/inet.h>

SubscriptionData::SubscriptionData(const uint16_t &_mid, 
                    const uint8_t &_qos_priority, 
                    const uint8_t &_qos_reliability)
{
  mid = _mid;
  qos_priority = _qos_priority;
  qos_reliability = _qos_reliability;
}                    


SbnPeer::SbnPeer(const std::string &_peer_ip,
                 const int &_peer_port,
	         const uint32_t &_peer_spacecraft_id, 
	         const uint32_t &_peer_processor_id,
	         const uint32_t &_spacecraft_id, 
	         const uint32_t &_processor_id)
{
   this->udp_ip = _peer_ip;
   this->udp_port = _peer_port;
   this->peer_spacecraft_id = _peer_spacecraft_id;
   this->peer_processor_id = _peer_processor_id;
   
   // Local
   this->spacecraft_id = _spacecraft_id;
   this->processor_id = _processor_id;
   
   // Socket structure
   memset(&this->address, 0, sizeof(this->address));

   this->address.sin_family = AF_INET;
   this->address.sin_addr.s_addr = inet_addr(this->udp_ip.c_str());
   this->address.sin_port = htons(this->udp_port);
      
   // Comm data
   this->connected = false;   
   //this->last_heartbeat_rx = node->now();
   //this->last_tx = node->now()
}
 


bool SbnPeer::hasRosSubscription(const uint16_t &_mid)
{
   for(auto si : this->ros_subscriptions)
      if(_mid == si)
        return true;
   
   return false;
}

void SbnPeer::addRosSubscription(const uint16_t &_mid)
{
   this->ros_subscriptions.push_back(_mid);
}

bool SbnPeer::hasSubscription(const uint16_t &_mid)
{
   for(auto si : this->subscriptions)
      if(si.mid == _mid)
        return true;
 
   return false;
}

/**
 * @function addSubscription
 */
size_t SbnPeer::addSubscriptions(const std::vector<SubscriptionData> &_subscriptions)
{
   for(auto si : _subscriptions)
   {
      if( !hasSubscription(si.mid) )
        this->subscriptions.push_back(si);
   }
   
   return this->subscriptions.size();
}

/**
 * @function deleteSubscription
 */
size_t SbnPeer::deleteSubscriptions(const std::vector<SubscriptionData> &_subscriptions)
{
   int idx;
   for(auto si : _subscriptions)
   {
      idx = -1;
      for(int i = 0; i < this->subscriptions.size(); ++i)
      {
         if(si.mid == this->subscriptions[i].mid)
         {
           idx = i; 
           break;
         }
      }
      
      if(idx != -1)
	this->subscriptions.erase(this->subscriptions.begin() + idx);      
   }
   
   return this->subscriptions.size();
}


/**
 * @function connected
 */
bool SbnPeer::setConnected( const rclcpp::Time &_now)
{
   this->last_heartbeat_rx = _now;

   if(!this->connected)
     this->connected = true;

   return this->connected;
}

