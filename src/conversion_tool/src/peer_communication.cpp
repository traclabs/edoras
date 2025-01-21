/**
 * @file peer_communication.cpp
 */
#include <conversion_tool/peer_communication.h>
#include <conversion_tool/memory_utils.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <cstdlib>
 
 
// Static 
const char*  PeerCommunication::rev_id_string_ = "$Id: dccf6239093d99c4c9351e140c15b61a95d8fc37 $";
const size_t PeerCommunication::rev_id_size_ = 48;
const size_t PeerCommunication::EDORAS_SBN_HDR_SIZE = 11;
const size_t PeerCommunication::EDORAS_TLM_HDR_SIZE = 16; // // tlm header: 8 primary + 4 secondary + 4 padding
/**
 * @function PeerCommunication
 * @brief Constructor
 */
PeerCommunication::PeerCommunication(std::shared_ptr<rclcpp::Node> _node)
{
 node_ = _node;
} 

 
/**
 * @function initialize
 * @brief Creates socket and binds to address
 */ 
bool PeerCommunication::initialize( const int &_udp_receive_port, 
			            const std::string &_udp_ip,
			            const uint32_t & _spacecraft_id, 
	                            const uint32_t & _processor_id,
			            std::string &_error_msg)
{
    // Store data
    udp_ip_ = _udp_ip;
    udp_receive_port_ = _udp_receive_port;
    spacecraft_id_ = _spacecraft_id;
    processor_id_ = _processor_id;
    
    // Create socket
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd_ < 0)
    {
       _error_msg = "Socket creation failed";
       return false;
    }
  
    memset(&own_address_, 0, sizeof(own_address_));
  
    // Fill server information
    own_address_.sin_family = AF_INET;
    own_address_.sin_addr.s_addr = inet_addr(udp_ip_.c_str());
    own_address_.sin_port = htons(udp_receive_port_);

  
    // Bind the socket
    int res = bind(sock_fd_, (struct sockaddr*)&own_address_, sizeof(own_address_));
    if( res < 0 )
    {
       _error_msg = "Bind failed";
       return false;
    }

    return true;
}  

/**
 * @function find
 */
bool PeerCommunication::findPeer(const uint32_t &_peer_spacecraft_id, 
                                 const uint32_t &_peer_processor_id)
{
  if( getPeer(_peer_spacecraft_id, _peer_processor_id) == nullptr)
    return false;
    
  else 
    return true;
}

/**
 * @function getPeer
 */
SbnPeer* PeerCommunication::getPeer(const uint32_t &_peer_spacecraft_id, 
                                    const uint32_t &_peer_processor_id)
{ 
  for(auto &pi : peers_)
  { 
     if(pi.peer_spacecraft_id == _peer_spacecraft_id && pi.peer_processor_id == _peer_processor_id)
       return &pi; 
  }
  
  return nullptr;
}

/**
 * @function addPeer
 */
bool PeerCommunication::addPeer(const std::string &_peer_ip,
                                const int &_peer_port,
	                        const uint32_t & _peer_spacecraft_id, 
	                        const uint32_t & _peer_processor_id)
{
   // If it exists, don't try to add or update
   if( findPeer(_peer_spacecraft_id, _peer_processor_id))
   {
      return false;
   }  
   
   SbnPeer peer = SbnPeer(_peer_ip, _peer_port, _peer_spacecraft_id, _peer_processor_id, 
                             this->spacecraft_id_, this->processor_id_);
   peers_.push_back(peer); 
   return true;
}	      

/**
 * @function sendAllSubscriptionMsg
 */
void PeerCommunication::sendAllSubscriptionMsg(const uint16_t &_mid)
{
   for(auto pe : peers_)
   {
      if( !pe.hasRosSubscription(_mid) )
      {
         pe.addRosSubscription(_mid);
         this->sendSubscriptionMsg(&pe, _mid);
      }
   }
}


void PeerCommunication::sendAllUnsubscriptionMsg(const uint16_t &_mid)
{

}





  
/**
 * @function receiveTlmPacket
 * @brief Received buffer has:
 * @brief  * TlmHeader (16 bytes) + 
 * @brief  * buffer_length (8 bytes) + 
 * @brief  * buffer_capacity (8 bytes) + 
 * @brief  * msg_serialized
 * @brief Return: True if the package received is a telemetry package (CFE)
 */  
bool PeerCommunication::receiveTlmPacket(uint16_t &_mid, uint8_t** _buffer)
{
   size_t offset = 6;
   ssize_t buffer_rcvd_size; 
   const int MAXLINE = 4096;
   uint8_t buffer_rcvd[MAXLINE];
   uint8_t* pointer = &buffer_rcvd[0];
   
   buffer_rcvd_size = recvfrom(sock_fd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
   if(buffer_rcvd_size > (ssize_t) offset)
   { 
      int sbn_type = handleSbnMsg(pointer, buffer_rcvd_size, _mid, _buffer);
      if(sbn_type == 3)
        return true;
        
   } // if buffer_rcvd_size > offset

   return false;  
}


/** 
 * @function updatePeer
 */
bool PeerCommunication::updatePeer(SbnPeer* _peer)
{
   _peer->setConnected(node_->now());
   
   if( _peer->ros_subscriptions.size() > 0 )
     return sendSubscriptionMsg(_peer, _peer->ros_subscriptions);
   else
     return sendProtocolMsg(_peer);

   //std::string msg = "Connected to Peer " + std::to_string(peer_spacecraft_id) + ":" + std::to_string(peer_processor_id);
}


/**
 * @function handleSbnMsg
 * @brief Return -1 if an error happens. If a valid message is processed, return sbn_type
 */
int PeerCommunication::handleSbnMsg(uint8_t* _buf, const ssize_t &_buf_size,
                                    uint16_t &_mid, uint8_t** _buffer_tlm)
{
   uint32_t spacecraft_id;
   uint32_t processor_id;
   uint16_t msg_size;
   uint8_t sbn_type;
   SbnPeer* peer = nullptr;

   if( readSbnHeader(_buf, _buf_size, msg_size, sbn_type, spacecraft_id, processor_id )  == 0)
     return -1;

   peer = getPeer(spacecraft_id, processor_id);
   if(peer == nullptr)
     return -1;
     
   // Update connection/heartbeat status for this peer
   updatePeer(peer);
   RCLCPP_INFO(node_->get_logger(), "DEBUG -- Handling sbn message of type: %02x", sbn_type);
   switch(sbn_type)
   {
      // Subscription message
      case 1:
      this->processSbnSubscriptionMsg(_buf, peer);
      break;
      
      // Unsubscription message
      case 2:
      processSbnUnsubscriptionMsg(_buf, peer);
      break;
      
      // SBN CFE message transfer
      case 3:
      processCfeTlmMessage(_buf, peer, _buf_size, _mid, _buffer_tlm);
      break;

      // SBN Protocol message      
      case 4:
        this->processSbnProtocolMsg(_buf);
        break;
      
      // SBN_UDP_HEARTBEAT_MSG
      // Send a heartbeat back
      case 0xA0:
        this->sendHeartbeat(peer);
        break;
      
      // SBN_UDP_ANNOUNCE_MSG
      case 0xA1:
      break;
      
      // SBN_UDP_DISCONN_MSG
      case 0xA2:
      break;
      
      default:
      {  printf("ERROR: Unknown message type!: %d \n", sbn_type);
         return -1;
      }
   }   
   
   return sbn_type;
}

/**
 * @function processSbnSubscriptionMsg
 */
void PeerCommunication::processSbnSubscriptionMsg(uint8_t *_buf, SbnPeer* _peer)
{  
   // Get git_id and subscriptions
   std::vector<SubscriptionData> subscriptions;
   parseSbnSubMsg(_buf, subscriptions);
   _peer->addSubscriptions(subscriptions);
}

/**
 * @function processSbnUnsubscriptionMsg
 * @brief Unsubscription messages are the same as subscription messages.
 */
void PeerCommunication::processSbnUnsubscriptionMsg(uint8_t *_buf, SbnPeer* _peer)
{  
   // Get git_id and subscriptions
   std::vector<SubscriptionData> subscriptions;
   parseSbnSubMsg(_buf, subscriptions);
   _peer->deleteSubscriptions(subscriptions);
}


/**
 * @function processSbnProtocolMsg
 * @brief Packet consists of SBN_HEADER + protocol (1 byte)
 */
void PeerCommunication::processSbnProtocolMsg(uint8_t *_buf)
{  
   RCLCPP_WARN(node_->get_logger(), "Process sbn protocol!");
   size_t offset = EDORAS_SBN_HDR_SIZE;
    
   // Read protocol
   uint8_t protocol;
   read_lb_uint8(_buf, offset, protocol);

   RCLCPP_WARN(node_->get_logger(), "Process sbn protocol: %u", protocol);
}

/**
 * @function processCfeTlmMesage
 * @brief Packet consists of SBN_HDR + TLM_HDR (16-byte header) + data (buffer_length + buffer_capacity + serialized)
 */
bool PeerCommunication::processCfeTlmMessage(uint8_t *_buf, SbnPeer* _peer, const ssize_t &_buf_size, 
                                             uint16_t &_mid, uint8_t** _buffer_tlm)
{
   // Get mid: First 2 bytes
   size_t offset = EDORAS_SBN_HDR_SIZE;
   _mid = ((uint16_t)_buf[offset] << 8) | _buf[offset + 1];
   
   // Get message data
   offset += EDORAS_TLM_HDR_SIZE;

   size_t buffer_tlm_size = (size_t) _buf_size - offset;
   *_buffer_tlm = static_cast<uint8_t *>( malloc(buffer_tlm_size) );
   memcpy(*_buffer_tlm, _buf + offset, buffer_tlm_size);

   return true;
}



void PeerCommunication::parseSbnSubMsg(uint8_t* _buf,
                                       std::vector<SubscriptionData> &_subscriptions)
{
  size_t offset = EDORAS_SBN_HDR_SIZE;
  size_t sz;
    
  // Read the first 48 bytes as git_id
  offset += 48;
  
  // Read subscription count
  uint16_t subscription_count;
  sz = read_lb_uint16(_buf, offset, subscription_count);
  offset += sz;

  _subscriptions.clear();  
  for(uint16_t i = 0; i < subscription_count; ++i)
  {
     uint32_t mid_32; uint16_t mid;
     uint8_t qos_priority;
     uint8_t qos_reliability;
     
     sz = read_lb_uint32(_buf, offset, mid_32);
     offset += sz;
     
     sz = read_lb_uint8(_buf, offset, qos_priority);
     offset += sz;
     
     sz = read_lb_uint8(_buf, offset, qos_reliability);
     offset += sz;

     mid = (uint16_t)mid_32;
     SubscriptionData si(mid, qos_priority, qos_reliability);
     _subscriptions.push_back(si);
  }
  
  return;
}



/**
 * @function readSbnHeader
 */
size_t PeerCommunication::readSbnHeader(uint8_t* _buf, 
                   const uint8_t &_buf_size,
                   uint16_t &_msg_size,
                   uint8_t &_msg_type,
                   uint32_t &_spacecraft_id, 
                   uint32_t &_processor_id)
{
   if(_buf_size < EDORAS_SBN_HDR_SIZE)
     return 0;
     
   size_t offset = 0;
   size_t sz;
      
   // Read message size (2)
   sz = read_lb_uint16(_buf, offset, _msg_size);
   offset += sz;

   // Read message type (1)
   sz = read_lb_uint8(_buf, offset, _msg_type);
   offset += sz;
      
   // Read processor_id (4)
   sz = read_lb_uint32(_buf, offset, _processor_id);
   offset += sz;
   
   // Read spacecraft id (4)
   sz = read_lb_uint32(_buf, offset, _spacecraft_id);
   offset += sz;
   RCLCPP_INFO(node_->get_logger(), "* Msg size: %d sbn msg type: %02x processor id: %d spacecraft id: %d . Buff size: %d", 
               _msg_size, _msg_type, _processor_id, _spacecraft_id, _buf_size);
   
   return offset;
}                   



/**
 * @function sendSubscriptionMsg
 * @brief Send subscription request to this peer
 */
bool PeerCommunication::sendSubscriptionMsg(SbnPeer* _peer, 
                                            const uint16_t &_mid)
{
  std::vector<uint16_t> mids;
  mids.push_back(_mid);
  return sendSubscriptionMsg(_peer, mids);  
}

/**
 * @function sendSubscriptionMsg
 * @brief Send subscription request to this peer
 */
bool PeerCommunication::sendSubscriptionMsg(SbnPeer *_peer, 
                                            const std::vector<uint16_t> &_mids)
{
   size_t msg_size;
   uint8_t msg_type = 1;
   uint8_t sbn_sub_qos_priority = 0;
   uint8_t sbn_sub_qos_reliability = 0;
   
   uint8_t* sub_msg = NULL;
   sub_msg = createSubscriptionMsg(msg_size, msg_type, _mids,
             sbn_sub_qos_priority, sbn_sub_qos_reliability);
             
   bool res = this->send(sub_msg, msg_size, _peer);
   
   // Cleanup
   free(sub_msg);
   
   return res;
}            


/**
 * @function sendProtocolMsg
 */
bool PeerCommunication::sendProtocolMsg(SbnPeer* _peer)
{
   size_t msg_size_minus_header = 1;  // VERIFY: this doesn't look right. 
   size_t msg_size = msg_size_minus_header + EDORAS_SBN_HDR_SIZE;
   uint8_t msg_type = 4;
   uint8_t protocol_id = 11;

   uint8_t* protocol_msg = static_cast<uint8_t *>( malloc(msg_size) );
   
   size_t offset = 0;
   size_t sz;
   
   sz = writeSbnHeader(&protocol_msg + offset, (uint16_t) msg_size_minus_header, msg_type);
   offset += sz;
   
   sz = memcpy_lb_uint8(&protocol_msg, offset, protocol_id);
   offset += sz;

   bool res = this->send(protocol_msg, msg_size, _peer);
   
   // Cleanup
   free(protocol_msg);
   
   return res;
}

/** 
 * @function sendHeartbeat
 * @brief Message: SBN_HEADER (11 bytes) with msg type == heartbeat 
 */
bool PeerCommunication::sendHeartbeat(SbnPeer* _peer)
{
   size_t msg_size_minus_header = 0;
   size_t msg_size = msg_size_minus_header + EDORAS_SBN_HDR_SIZE;

   uint8_t* heartbeat_msg = static_cast<uint8_t *>( malloc(msg_size) );
   uint8_t msg_type = 0xA0;

   writeSbnHeader(&heartbeat_msg, (uint16_t)msg_size_minus_header, msg_type);
   
   bool res = this->send(heartbeat_msg, msg_size, _peer);
   
   // Cleanup
   free(heartbeat_msg);
   
   return res;
}


/**
 * @function send
 */
bool PeerCommunication::send(uint8_t* _packet, 
                             const size_t &_packet_size, 
                             SbnPeer* _peer)
{
   _peer->last_tx = node_->now();
   int res = sendto(sock_fd_, _packet, _packet_size, 0, (const struct sockaddr *)&_peer->address, sizeof(_peer->address));      
   return res < 0 ? false : true;
}

////////////////////////////////
// CREATE HELPERS             //
////////////////////////////////

/**
 * @function creatSubscriptionMsg
 */
uint8_t* PeerCommunication::createSubscriptionMsg(size_t &_msg_size,
                           const uint8_t &_msg_type,
                           const std::vector<uint16_t> &_mids, 
                           const uint8_t &_sbn_sub_qos_priority,
                           const uint8_t &_sbn_sub_qos_reliability) 
{
  // 50: 48 (git_id) + 2 (sbn_sub_count)
  // 6: 4 (mid) + 1 (qos_priority) + 1 (qos_reliability) 
  uint16_t sbn_sub_count = _mids.size();

  size_t msg_size_minus_header = 50 + (sbn_sub_count) * 6;
  _msg_size = msg_size_minus_header + EDORAS_SBN_HDR_SIZE; 

  uint8_t* sub_msg = static_cast<uint8_t *>( malloc(_msg_size) );

  size_t offset = 0;
  size_t sz;
  
  // SBN Header: 11 bytes
  sz = writeSbnHeader(&sub_msg + offset, (uint16_t)msg_size_minus_header, _msg_type);
  offset += sz; 

  // git_id: 48 bytes
  char rev_id[48] = "$Id: dccf6239093d99c4c9351e140c15b61a95d8fc37 $";
  memcpy(sub_msg + offset, &rev_id, this->rev_id_size_);  
  offset += rev_id_size_;

  // sbn_sub_count: 2 bytes
  sz = memcpy_lb_uint16(&sub_msg, offset, sbn_sub_count);
  offset += sz;

  // For each sub_count : mid(4) + qos_p(1) + qos_r(1) =  6 bytes
   for(auto mi : _mids)
   {
      // cFS/cfe/modules/core_api/fsw/inc/cfe_sb_extern_typedefs.h
      // mid is of size 2 but should be stored as 4 (uint32)
      uint32_t mid_32 = static_cast<uint32_t>(mi);
      sz = memcpy_lb_uint32(&sub_msg, offset, mid_32);
      offset += sz;
  
      sz = memcpy_lb_uint8(&sub_msg, offset, _sbn_sub_qos_priority);
      offset += sz; 

      sz = memcpy_lb_uint8(&sub_msg, offset, _sbn_sub_qos_reliability);
      offset += sz; 
   }
  
  // Double-check
  if(offset != _msg_size)
    RCLCPP_INFO(node_->get_logger(), "Create subscription message error: Msg size: %lu, offsets: %lu  rev id size: %lu \n", _msg_size, offset, rev_id_size_);
  
  return sub_msg;   
}


/**
 * @function writeSbnHeader
 */
size_t PeerCommunication::writeSbnHeader(uint8_t** _msg, 
                                         const uint16_t &_msg_size, 
                                         const uint8_t &_msg_type)
{
   size_t offset = 0;
   size_t sz;
   
   // 2 Bytes: msg_size (uint16_t)
   sz = memcpy_lb_uint16(_msg, offset, _msg_size);
   offset += sz;
   
   // 1 Byte: msg_type   
   sz = memcpy_lb_uint8(_msg, offset, _msg_type);
   offset += sz;
   
   // 4 Bytes: processor ID
   sz = memcpy_lb_uint32(_msg, offset, this->processor_id_);
   offset += sz;

   // 4 Bytes: spacecraft id:
   sz = memcpy_lb_uint32(_msg, offset, this->spacecraft_id_);
   offset += sz;
   
   return offset;
}

 
/**
 * @function sendCmdPacket
 */
 /*
bool PeerCommunication::sendCmdPacket(const uint16_t &_mid, const uint8_t &_code, 
                                       const uint16_t &_seq, unsigned char** _data_buffer, 
                                       const size_t &_data_size)
{   
    //size_t         bufSize = serialize(_js, &buf);
    size_t header_ccsds_offset = 8;
    unsigned char* cmd_packet = new unsigned char[_data_size + header_ccsds_offset];
    size_t cmd_packet_size;
    cmd_packet_size = createCmdPacket(_mid, _code, _seq, _data_size, _data_buffer, &cmd_packet);
    
    //  Read data from cmd packet
    // DEBUG START----
    size_t buffer_length, buffer_capacity;
    memcpy(&buffer_length, cmd_packet + header_ccsds_offset, sizeof(size_t));
    memcpy(&buffer_capacity, cmd_packet + header_ccsds_offset + sizeof(size_t), sizeof(size_t));
    //printf("sendCmdPacket: Packet length: %d Buffer length: %ld -- capacity: %ld ! \n", cmd_packet_size, buffer_length, buffer_capacity); 
    // DEBUG END -----
    int res = sendto(sock_fd_, cmd_packet, cmd_packet_size, 0, (const struct sockaddr *)&fsw_address_, sizeof(fsw_address_));
    
    if(res < 0)
      return false;

    return true;
}*/

/**
 * @function createCmdPacket
 * @brief Add command header on top of data buffer and return it
 */
 /*
size_t PeerCommunication::createCmdPacket(const uint16_t &_mid, const uint8_t &_code, const uint16_t &_seq, 
                                           const size_t &_data_size, unsigned char** _data_buffer, unsigned char** _cmd_packet)
{
   // Header contains
   // 2 bytes: stream_id
   // 2 bytes: sequence: 
   // 2 bytes: length
   // 1 bytes: code
   // 1 bytes: checksum
   // 2 + 2 + 2 + 1 + 1 = 8 bytes
   const uint8_t header_size = 8;
   uint16_t packet_length = header_size + _data_size;
   uint16_t packet_length_ccsds_offset = packet_length - 7;
   
   unsigned char  cmd_header[header_size];

   cmd_header[0] = (_mid >> 8) & 0xFF;
   cmd_header[1] = _mid & 0xFF;
   cmd_header[2] = (_seq >> 8) & 0xFF;
   cmd_header[3] = _seq & 0xFF;
   cmd_header[4] = (packet_length_ccsds_offset >> 8) & 0xFF;
   cmd_header[5] = packet_length_ccsds_offset & 0xFF;
   cmd_header[6] = _code;
   cmd_header[7] = 0;
   
   // DEBUG
   //printf("BC: createCmdPacket: Header: ");
   // for (uint8_t i = 0; i < header_size; i++) {
  //      printf("%02x ", cmd_header[i]);
   // } printf("\n");

   // Add header bytes on top of the serialized data
   size_t offset = 0;
  
   memcpy(*_cmd_packet + offset, &cmd_header, sizeof(cmd_header));  offset += sizeof(cmd_header);
   memcpy(*_cmd_packet + offset, *_data_buffer, _data_size);
   
  return packet_length;
}*/


