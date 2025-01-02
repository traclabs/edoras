/** 
 * @file parser_utils.h
 */
#pragma once
  
#include <conversion_tool/types.h>

#include <string>
#include <rclcpp/rclcpp.hpp>

/**
 * @function from_serialized_to_byte_array
 * @brief Converts from a SerializedMessage data to data formated in a C message structure
 */
uint8_t* from_serialized_to_byte_array(const rcl_serialized_message_t* _serialized_msg,
                                       const TypeSupport_t* _type_support,
                                       const TypeInfo_t* _type_info,
                                       size_t &_buffer_size, 
                                       std::string &_error_msg);

uint8_t* from_rcutils_uint_array_to_uint_buffer(const rcl_serialized_message_t* _serialized_msg, 
                                                size_t &_buffer_size, size_t &_msg_length, size_t&_msg_capacity);


rcutils_uint8_array_t* make_serialized_array(const uint8_t* _buffer, size_t &_bl, size_t &_bc);

const TypeInfo_t * get_type_info(const std::string  &_interface_name, 
                                 const std::string &_interface_type,
                                 std::string &_error_msg);

bool extract_type_identifier(const std::string & full_type, 
                             std::string &_interface_name, 
                             std::string &_interface_type);

uint8_t* from_serialized_to_byte_array_2(const rcl_serialized_message_t* _serialized_msg,
                                       const TypeSupport_t* _type_support,
                                       const TypeInfo_t* _type_info,
                                       size_t &_buffer_size, 
                                       std::string &_error_msg);
