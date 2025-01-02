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
			             const std::string &_telemetry_ip,
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
    own_address_.sin_addr.s_addr = inet_addr(_telemetry_ip.c_str()); //"127.0.0.1" INADDR_ANY;
    own_address_.sin_port = htons(_own_port);


    // Fill fsw information (fsw: Where cFS runs)
    fsw_address_.sin_family = AF_INET;
    fsw_address_.sin_addr.s_addr = inet_addr(_fsw_ip.c_str()); //INADDR_ANY;
    fsw_address_.sin_port = htons(_fsw_port);
  
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
    printf("sendCmdPacket: Packet length: %d Buffer length: %ld -- capacity: %ld ! \n", cmd_packet_size, buffer_length, buffer_capacity); 
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
   
   printf("BC: createCmdPacket: Header: ");
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
 * @brief Received buffer has:
 * @brief  * TlmHeader (16 bytes) + 
 * @brief  * buffer_length (8 bytes) + 
 * @brief  * buffer_capacity (8 bytes) + 
 * @brief  * msg_serialized
 */  
bool BasicCommunication::receiveTlmPacket(uint16_t &_mid, uint8_t** _buffer, 
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
         
      return true;
      
   } // if buffer_rcvd_size > 0

   return false;  
}




