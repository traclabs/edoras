#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cstdint>

uint8_t* from_rcutils_uint_array_to_uint_buffer(const rcl_serialized_message_t* _serialized_msg, 
                                                size_t &_buffer_size, size_t &_msg_length,
                                                size_t&_msg_capacity);
                                                
rcutils_uint8_array_t* make_serialized_array(const uint8_t* _buffer);                                                
