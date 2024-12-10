/**
 * @file parser_utils.h
 */
#include <conversion_tool/parser_utils.h>
#include <sstream>
#include <dlfcn.h>

uint8_t* from_serialized_to_byte_array(const rcl_serialized_message_t* _serialized_msg,
                                       std::shared_ptr<rcpputils::SharedLibrary> _library, 
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
 *
 */
void debug_read_message(uint8_t* _data_buffer, const TypeInfo_t *_type_info)
{ 
  for (uint32_t i = 0; i < _type_info->member_count_; ++i) {
    // Get the introspection information for this particular member
    const MemberInfo_t & member_info = _type_info->members_[i];
    // Get a pointer to the member's data in the binary buffer
    uint8_t * member_data = &_data_buffer[member_info.offset_];

    // Recursively (because some members may be non-primitive types themselves) convert the member
    //yaml_msg[member_info.name_] = dynmsg::c::impl::member_to_yaml(member_info, member_data);
    member_to_yaml(member_info, member_data);
    RCLCPP_INFO(rclcpp::get_logger("parser_utils"), "Member[%lu] offset: %d -- Member info name: %s \n", i, member_info.offset_, member_info.name_);
  }

}

void member_to_yaml(const MemberInfo_t & _member_info, uint8_t * _member_data)
{  
  if (_member_info.is_array_) {

    if (_member_info.is_upper_bound_ || _member_info.array_size_ == 0) {
      dynamic_array_to_yaml( _member_info, _member_data);
    }
  }
  
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


//////////////////

// Convert the binary data for an individual element of an array member to YAML
void member_to_yaml_array_item(const MemberInfo_t & member_info,
                               const uint8_t * member_data)
{
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      //array_node.push_back(*reinterpret_cast<const float *>(member_data));
      RCLCPP_INFO(rclcpp::get_logger("parser_utils"), "Member float: %f ", *reinterpret_cast<const float *>(member_data) );
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      //array_node.push_back(*reinterpret_cast<const double *>(member_data));
      RCLCPP_INFO(rclcpp::get_logger("parser_utils"), "Member double: %f -- address: %p", *reinterpret_cast<const double *>(member_data), (void*)(&member_data) );
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      //array_node.push_back(*reinterpret_cast<const long double *>(member_data));
      RCLCPP_INFO(rclcpp::get_logger("parser_utils"), "Member long double: %f ", *reinterpret_cast<const long double *>(member_data) );
      break;
    /*case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      //array_node.push_back(
      //  std::string(reinterpret_cast<const rosidl_runtime_c__String *>(member_data)->data));
      RCLCPP_INFO(_node->get_logger(), "Member: %s ", *reinterpret_cast<const rosidl_runtime_c__String* *>(member_data)->data );

      break;*/

    default:
      //array_node.push_back("Unknown value for unknown type");
      break;
  }
}

// Convert an individual member's value from binary to YAML
/*
void basic_value_to_yaml(
  const MemberInfo & member_info,
  const uint8_t * member_data,
  YAML::Node & member)
{
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      member["value"] = *reinterpret_cast<const float *>(member_data);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      member["value"] = *reinterpret_cast<const double *>(member_data);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      member["value"] = *reinterpret_cast<const long double *>(member_data);
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      member["value"] = reinterpret_cast<const rosidl_runtime_c__String *>(member_data)->data;
      break;
 
    default:
      member["value"] = "Unknown value for unknown type";
      break;
  }
}*/



// Convert a dynamically-sized sequence to YAML
void
dynamic_array_to_yaml(const MemberInfo_t & member_info,
                      const uint8_t * member_data)
{
  switch (member_info.type_id_) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__float__Sequence *>(member_data) );
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__double__Sequence *>(member_data) );
      break;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__long_double__Sequence *>(member_data) );
      break;
 
    case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
      dynamic_array_to_yaml_impl(member_info,
        reinterpret_cast<const rosidl_runtime_c__String__Sequence *>(member_data));
      break;

    default:
      // Store an error string and keep going
      //array_node.push_back("Unknown value for unknown type");
      break;
  }
}

