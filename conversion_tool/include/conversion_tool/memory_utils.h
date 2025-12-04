#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

size_t memcpy_lb_uint32(uint8_t** _buf, const size_t &_offset, const uint32_t &_data);
size_t memcpy_lb_uint16(uint8_t** _buf, const size_t &_offset, const uint16_t &_data);
size_t memcpy_lb_uint8(uint8_t** _buf, const size_t &_offset, const uint8_t &_data);

size_t read_lb_uint8(uint8_t* _buf, const size_t &_offset, uint8_t &_data);
size_t read_lb_uint16(uint8_t* _buf, const size_t &_offset, uint16_t &_data);
size_t read_lb_uint32(uint8_t* _buf, const size_t &_offset, uint32_t &_data);


// Helper functions
std::string getBufferString(uint8_t* _buffer, size_t _buffer_size);
