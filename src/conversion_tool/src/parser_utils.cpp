/**
 * @file parser_utils.h
 */
#include <conversion_tool/parser_utils.h>
#include <sstream>
#include <dlfcn.h>

/**
 * @function get_type_info
 */
const rosidl_typesupport_introspection_c__MessageMembers *
  get_type_info(const std::string  &_interface_name, 
                const std::string &_interface_type, 
                std::string &_error_msg)
{
  // Load the introspection library for the package containing the requested type
  std::stringstream ts_lib_name;
  ts_lib_name << "lib" << _interface_name << "__rosidl_typesupport_introspection_c.so";

  void * introspection_type_support_lib = dlopen(ts_lib_name.str().c_str(), RTLD_LAZY);
  if (introspection_type_support_lib == nullptr) {
    _error_msg = "Failed to load introspection type support library: " + std::string(dlerror());
    return nullptr;
  }
  
  // Load the function that, when called, will give us the introspection information for the
  // interface type we are interested in
  std::stringstream ts_func_name;
  ts_func_name << "rosidl_typesupport_introspection_c__get_message_type_support_handle__" <<
    _interface_name << "__msg__" << _interface_type;

  get_message_ts_func introspection_type_support_handle_func =
    reinterpret_cast<get_message_ts_func>(dlsym(
      introspection_type_support_lib,
      ts_func_name.str().c_str()));
  if (introspection_type_support_handle_func == nullptr) {
    _error_msg = "failed to load introspection type support function: " + std::string(dlerror());
    return nullptr;
  }

  // Call the function to get the introspection information we want
  const rosidl_message_type_support_t * introspection_ts =
    introspection_type_support_handle_func();

  const rosidl_typesupport_introspection_c__MessageMembers * type_info =
    reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
    introspection_ts->data);

  return type_info;
}

/**
 * @function extract_type_identifier
 */
bool extract_type_identifier(const std::string & full_type, 
                             std::string &_interface_name, 
                             std::string &_interface_type)
{
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (sep_position_back == std::string::npos ||
    sep_position_back == 0 ||
    sep_position_back == full_type.length() - 1)
  {
    //error = "Message type is not of the form package/type and cannot be processed");
    return false;
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module =
      full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  _interface_name = package_name;
  _interface_type = type_name;
  
  return true;
}
