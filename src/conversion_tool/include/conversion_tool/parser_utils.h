/** 
 * @file parser_utils.h
 */
#pragma once
  
#include <conversion_tool/types.h>

#include <rosidl_typesupport_introspection_c/field_types.h>
#include <string>
#include <rclcpp/rclcpp.hpp>

/**
 * @function from_serialized_to_byte_array
 * @brief Converts from a SerializedMessage data to data formated in a C message structure
 */
uint8_t* from_serialized_to_byte_array(const rcl_serialized_message_t* _serialized_msg,
                                       std::shared_ptr<rcpputils::SharedLibrary> _library, 
                                       const TypeSupport_t* _type_support,
                                       const TypeInfo_t* _type_info,
                                       size_t &_buffer_size, 
                                       std::string &_error_msg);


const TypeInfo_t * get_type_info(const std::string  &_interface_name, 
                                 const std::string &_interface_type,
                                 std::string &_error_msg);

bool extract_type_identifier(const std::string & full_type, 
                             std::string &_interface_name, 
                             std::string &_interface_type);
                             
void dynamic_array_to_yaml(const MemberInfo_t & member_info,
                           const uint8_t * member_data);

void member_to_yaml_array_item(const MemberInfo_t & member_info,
                               const uint8_t * member_data);

void member_to_yaml(const MemberInfo_t & _member_info, uint8_t * _member_data);
  
  // Convert a dynamically-sized sequence to YAML - implementation function
template<typename T>
void
dynamic_array_to_yaml_impl(const MemberInfo_t & member_info, T * array)
{
  for (size_t ii = 0; ii < array->size; ++ii) {
    member_to_yaml_array_item(member_info,
      reinterpret_cast<const uint8_t *>(&array->data[ii]));
  }
}

/////////////////
// DEBUG
void debug_read_message(uint8_t* _data_buffer, const TypeInfo_t *_type_info);

