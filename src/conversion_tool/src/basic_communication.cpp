/**
 * @file basic_communication.cpp
 */
#include <conversion_tool/basic_communication.h>
#include <stdio.h>
#include <cstdlib>
 
/**
 * @function BasicCommunication
 * @brief Constructor
 */
BasicCommunication::BasicCommunication()
{} 
 
/**
 * @function initialize
 * @brief Creates socket and binds to address
 */ 
bool BasicCommunication::initialize( const int &_own_port, 
			             const int &_fsw_port,
			             const std::string &_fsw_ip, 
			             std::string &_error_msg)
{
    // Create socket
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd_ < 0)
    {
       _error_msg = "Socket creation failed";
       return false;
    }
  
    memset(&own_address_, 0, sizeof(own_address_));
    memset(&fsw_address_, 0, sizeof(fsw_address_));
  
    // Fill server information
    own_address_.sin_family = AF_INET;
    own_address_.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
    own_address_.sin_port = htons(_own_port);


    // Fill fsw information (fsw: Where cFS runs)
    fsw_address_.sin_family = AF_INET;
    fsw_address_.sin_addr.s_addr = inet_addr(_fsw_ip.c_str()); //INADDR_ANY;
    fsw_address_.sin_port = htons(_fsw_port);
  
    // Bind the socket
    int res = bind(sock_fd_, (const struct sockaddr*)&own_address_, sizeof(own_address_));
    if( res < 0 )
    {
       _error_msg = "Bind failed";
       return false;
    }

    return true;
}  
  
/**
 * @function sendCmdPacket
 */
bool BasicCommunication::sendCmdPacket(const uint16_t &_mid, const uint8_t &_code, 
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
    printf("BEFORE SENDING THROUGH NETWORK VERIFY: Packet length: %d BUFFER LENGTH: %ld -- buffer capacity: %ld -- size of size_t: %d !!! \n", cmd_packet_size, buffer_length, buffer_capacity, sizeof(size_t)); 
    // DEBUG END -----
    int res = sendto(sock_fd_, cmd_packet, cmd_packet_size, 0, (const struct sockaddr *)&fsw_address_, sizeof(fsw_address_));
    
    if(res < 0)
      return false;

    return true;
}

/**
 * @function createCmdPacket
 * @brief Add command header on top of data buffer and return it
 */
size_t BasicCommunication::createCmdPacket(const uint16_t &_mid, const uint8_t &_code, const uint16_t &_seq, 
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
   
   printf("Printing header values \n");
    for (uint8_t i = 0; i < header_size; i++) {
        printf("%02x ", cmd_header[i]);
    } printf("\n");


   // Add header bytes on top of the serialized data
   //*_cmd_packet = (unsigned char*) calloc(1, packet_length);
   size_t offset = 0;
  
   memcpy(*_cmd_packet + offset, &cmd_header, sizeof(cmd_header));  offset += sizeof(cmd_header);
   memcpy(*_cmd_packet + offset, *_data_buffer, _data_size);
   
  return packet_length;
}

  
/**
 * @function receiveTlmPacket
 */  
bool BasicCommunication::receiveTlmPacket(size_t &_buffer_size, std::vector<uint8_t> &_header_debug )
{
   ssize_t buffer_size_rcvd; 
   const int MAXLINE = 1024;
   uint8_t buffer[MAXLINE];
   // Receive............
   // (unsigned char*)
   buffer_size_rcvd = recvfrom(sock_fd_, (uint8_t*) buffer, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
   if(buffer_size_rcvd > 0)
   { 
      _buffer_size = (size_t)buffer_size_rcvd;
      // DEBUG --------------------
      if(buffer_size_rcvd > 8)
      {
        _header_debug.clear();
        for(int i = 0; i < 8; ++i)
           _header_debug.push_back( buffer[i]);
      }
      // DEBUG -------------------
      return true;
   }

   return false;  
}




