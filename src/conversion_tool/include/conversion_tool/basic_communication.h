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

/**
 * @class BasicCommunication
 */
class BasicCommunication
{
 public:
 
 BasicCommunication();
 bool initialize(const int &_own_port, 
		 const int &_fsw_port,
		 const std::string &_fsw_ip, 
		 std::string &_error_msg);
 
 bool sendCmdPacket(const uint16_t &_mid, const uint8_t &_code, 
                    const uint16_t &_seq, unsigned char** _data_buffer, 
                    const size_t &_data_size);
 bool receiveTlmPacket(uint16_t &_mid, uint8_t** _buffer, 
                       std::vector<uint8_t> &_header_debug, 
                       std::vector<uint8_t> &_buffer_test,
                       size_t &_brs, size_t &_offset, size_t &_bs);
 
 protected:

  size_t createCmdPacket(const uint16_t &_mid, const uint8_t &_code, const uint16_t &_seq, 
                         const size_t &_data_size, unsigned char** _data_buffer, unsigned char** _cmd_packet);

  int sock_fd_;
  char buffer_[1024];
  struct sockaddr_in own_address_;
  struct sockaddr_in fsw_address_; 
};
