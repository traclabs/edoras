/** 
 * @file parser_utils.h
 */
#pragma once
  
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <string>

typedef const rosidl_message_type_support_t * (* get_message_ts_func)();

const rosidl_typesupport_introspection_c__MessageMembers * 
   get_type_info(const std::string  &_interface_name, 
                 const std::string &_interface_type,
                 std::string &_error_msg);

bool extract_type_identifier(const std::string & full_type, 
                             std::string &_interface_name, 
                             std::string &_interface_type);
