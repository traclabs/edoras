/**
 * @file parser_utils.h
 */
#include <conversion_tool/parser_utils.h>
#include <sstream>
#include <dlfcn.h>

uint8_t* from_serialized_to_byte_array(const rcl_serialized_message_t* _serialized_msg,
                                       const TypeSupport_t* _type_support,
                                       const TypeInfo_t* _type_info,
                                       size_t &_buffer_size, 
                                       std::string &_error_msg)
{    
  //uint8_t* data; // = reinterpret_cast<uint8_t *>(&ros_msg);

  std::string error_msg;

 
  // Allocate space for the pointer to the C data
  rcutils_allocator_t * allocator;
  rcutils_allocator_t default_allocator = rcutils_get_default_allocator();
  allocator = &default_allocator;

  // Get default buffer size
  _buffer_size = _type_info->size_of_;

  // Allocate space to store the binary representation of the message
  uint8_t * data = static_cast<uint8_t *>(allocator->allocate(_type_info->size_of_, allocator->state));

  if (data == nullptr) {
    _error_msg = "Error allocating space";
    return nullptr;
  }

  // Initialise the message buffer according to the interface type
  _type_info->init_function(data, ROSIDL_RUNTIME_C_MSG_INIT_ALL);

  // Deserialize
  if ( rmw_deserialize( _serialized_msg, _type_support, data ) != RMW_RET_OK ) {
    _error_msg = "Failed to apply rmw_deserialize";
    return nullptr;
  }
    
  return data;
}

/**
 * @function from_rcutils_uint_array_to_uint_buffer 
 * @brief
 * @output An array of bytes. It contains:
 * size_t (2 bytes): buffer length
 * size_t (2 bytes): buffer capacity
 * rest of bytes: Serialized data in format used by ROS2 by default (in this case, CDR, from fastrtps)
 * _buffer_size : Parameter that contains the size of the output buffer (4 + size_of (rest of bytes) )
 */
uint8_t* from_rcutils_uint_array_to_uint_buffer(const rcl_serialized_message_t* _serialized_msg, 
                                                size_t &_buffer_size, size_t &_msg_length, size_t&_msg_capacity)
{
   size_t msg_buffer_length = _serialized_msg->buffer_length;
   size_t msg_buffer_capacity = _serialized_msg->buffer_capacity;


   // Set the buffer_size
   _buffer_size = sizeof(size_t) + sizeof(size_t) + msg_buffer_length;
   
   // Get space for size_t + size_t + msg_buffer
   uint8_t* buffer = new uint8_t[_buffer_size]; 
   
   
   // Fill the space
   uint8_t offset = 0;
   std::memcpy(buffer + offset, &msg_buffer_length, sizeof(size_t));  
   offset += sizeof(msg_buffer_length);
   std::memcpy(buffer + offset, &msg_buffer_capacity, sizeof(size_t));
   offset += sizeof(msg_buffer_capacity);
   std::memcpy(buffer + offset, _serialized_msg->buffer, msg_buffer_length);

   // Return
   
   // DEBUG
   _msg_length = msg_buffer_length;
   _msg_capacity = msg_buffer_capacity;
   
   return buffer;
}


/**
 * @function get_type_info
 */
const TypeInfo_t *
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


