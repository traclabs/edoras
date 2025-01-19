/**
 * @file peer_communication.cpp
 */
#include <conversion_tool/peer_communication.h>
#include <stdio.h>
#include <cstdlib>
 
SbnPeer::SbnPeer(const std::string &_peer_ip,
                 const int &_peer_port,
	         const int &_peer_spacecraft_id, 
	         const int &_peer_processor_id,
	         const int &_spacecraft_id, 
	         const int &_processor_id)
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
   ros_subscriptions.push_back(_mid);
}
 
// Static 
const char*  PeerCommunication::rev_id_string_ = "$Id: dccf6239093d99c4c9351e140c15b61a95d8fc37 $";
const size_t PeerCommunication::rev_id_size_ = 48;
 
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
			            const int & _spacecraft_id, 
	                            const int & _processor_id,
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
bool PeerCommunication::find(const int &_peer_spacecraft_id, 
          const int &_peer_processor_id)
{
  for(auto pi : peers_)
  {
     if(pi.peer_spacecraft_id == _peer_spacecraft_id && pi.processor_id == _peer_processor_id)
       return true;
  }
  
  return false;
}

/**
 * @function addPeer
 */
bool PeerCommunication::addPeer(const std::string &_peer_ip,
                                const int &_peer_port,
	                        const int & _peer_spacecraft_id, 
	                        const int & _peer_processor_id)
{
   // If it exists, don't try to add or update
   if( find(_peer_spacecraft_id, _peer_processor_id))
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
         this->sendSubscriptionMsg(pe, _mid);
      }
   }
}

/**
 * @function sendSubscriptionMsg
 * @brief Send subscription request to this peer
 */
void PeerCommunication::sendSubscriptionMsg(SbnPeer &_peer, const uint16_t &_mid)
{
  size_t msg_size; // = 56;  // Base size of message with a single subscription
  uint8_t msg_type = 1;
  uint8_t sbn_sub_qos_priority = 0;
  uint8_t sbn_sub_qos_reliability = 0;
  uint16_t sbn_count = 1;
  
  // mid is a single ID.
  uint8_t* sub_msg = NULL;
  sub_msg = createSubscriptionMsg(msg_size, msg_type, sbn_count, _mid, 
                        sbn_sub_qos_priority, sbn_sub_qos_reliability, _peer);
          
  this->send(sub_msg, msg_size, _peer);
  
  // Cleanup
  free(sub_msg);
}

void PeerCommunication::sendAllUnsubscriptionMsg(const uint16_t &_mid)
{

}

/**
 * @function send
 */
bool PeerCommunication::send(uint8_t* _packet, 
                             const size_t &_packet_size, 
                             SbnPeer &_peer)
{
   _peer.last_tx = node_->now();
   int res = sendto(sock_fd_, _packet, _packet_size, 0, (const struct sockaddr *)&_peer.address, sizeof(_peer.address));      
   return res < 0 ? false : true;
}


/**
 * @brief Header: 11 bytes
 * @brief rev_id_string: 48 bytes
 * @brief sub_count: 2 bytes
 */
uint8_t* PeerCommunication::createSubscriptionMsg(size_t &_msg_size,
                           const uint8_t &_msg_type,
                           const uint16_t &_sbn_count, 
                           const uint16_t &_mid, 
                           const uint8_t &_sbn_sub_qos_priority,
                           const uint8_t &_sbn_sub_qos_reliability,
                           SbnPeer &_peer)
{ RCLCPP_INFO(node_->get_logger(), "Create subscription msg");
  
  size_t msg_size_minus_header = 56;
  _msg_size = msg_size_minus_header + 11; // 11 = Header size
   
  uint8_t* sub_msg = static_cast<uint8_t *>( malloc(_msg_size) );

  size_t offset = 0;
  size_t header_size = writeSbnHeader(&sub_msg + offset, (uint16_t)msg_size_minus_header, _msg_type);
  for(int i = 0; i < 11; ++i)
  {
     RCLCPP_INFO(node_->get_logger(), "Header= byte[%d] %02x ", i, *(sub_msg + i));
  }
  offset += header_size; 

  memcpy(sub_msg + offset, &this->rev_id_string_, this->rev_id_size_); // 48
  offset += rev_id_size_;
  
  memcpy(sub_msg + offset, &_sbn_count, sizeof(_sbn_count)); // 2
  offset += sizeof(_sbn_count);  

  // cFS/cfe/modules/core_api/fsw/inc/cfe_sb_extern_typedefs.h
  // mid is of size 2 but should be stored as 4 (uint32)
  uint32_t mid_32 = static_cast<uint32_t>(_mid);
  memcpy(sub_msg + offset, &mid_32, sizeof(mid_32)); // 4
  offset += sizeof(mid_32);

  memcpy(sub_msg + offset, &_sbn_sub_qos_priority, sizeof(_sbn_sub_qos_priority)); // 1
  offset += sizeof(_sbn_sub_qos_priority);

  memcpy(sub_msg + offset, &_sbn_sub_qos_reliability, sizeof(_sbn_sub_qos_reliability)); // 1
  offset += sizeof(_sbn_sub_qos_reliability);
  
  // Double-check
  if(offset - header_size != _msg_size)
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
   uint8_t data;
   size_t offset = 0;
   
   // 2 Bytes: msg_size (uint16_t)
   data = (_msg_size >> 8) & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   data = _msg_size & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);
   
   // 1 Byte: msg_type   
   memcpy(*_msg + offset, &_msg_type, sizeof(_msg_type));
   offset += sizeof(uint8_t);
   
   // 4 Bytes: processor ID
   data = (this->processor_id_ >> 24) & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   data = (this->processor_id_ >> 16) & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   data = (this->processor_id_ >> 8) & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   data = this->processor_id_ & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   // 4 Bytes: spacecraft id:
   data = (this->spacecraft_id_ >> 24) & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   data = (this->spacecraft_id_ >> 16) & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   data = (this->spacecraft_id_ >> 8) & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

   data = this->spacecraft_id_ & 0xFF;
   memcpy(*_msg + offset, &data, sizeof(uint8_t));
   offset += sizeof(uint8_t);

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

  
/**
 * @function receiveTlmPacket
 * @brief Received buffer has:
 * @brief  * TlmHeader (16 bytes) + 
 * @brief  * buffer_length (8 bytes) + 
 * @brief  * buffer_capacity (8 bytes) + 
 * @brief  * msg_serialized
 */
 /*  
bool PeerCommunication::receiveTlmPacket(uint16_t &_mid, uint8_t** _buffer, 
                                          std::vector<uint8_t> &_header_debug,
                                          size_t &_buffer_size )
{
   size_t offset = 16; // tlm header: 8 primary + 4 secondary + 4 padding
   ssize_t buffer_rcvd_size; 
   const int MAXLINE = 1024;
   uint8_t buffer_rcvd[MAXLINE];
   
   // Receive............
   // (unsigned char*)
   buffer_rcvd_size = recvfrom(sock_fd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
   if(buffer_rcvd_size > (ssize_t) offset)
   { 
      // DEBUG --------------------
        _header_debug.clear();
        for(int i = 0; i < 8; ++i)
           _header_debug.push_back( buffer_rcvd[i]);            
      // DEBUG -------------------
      
      // Get mid: First 2 bytes
      _mid = ((uint16_t)buffer_rcvd[0] << 8) | buffer_rcvd[1];
      
      // Get buffer
      _buffer_size = (size_t) buffer_rcvd_size - offset;
      
      *_buffer = static_cast<uint8_t *>( malloc(_buffer_size) );
      memcpy(*_buffer, buffer_rcvd + offset, _buffer_size);

      // See buffer contents
      /*for(int i = 0; i < _buffer_size; i++)
      { printf(" %02x ", *(*_buffer + i) );
        if(i % 8 == 7)
          printf("\n");
      } printf("\n");*/
         
   //   return true;
      
  // } // if buffer_rcvd_size > 0

  // return false;  
//}




