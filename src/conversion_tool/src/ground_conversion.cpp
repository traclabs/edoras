/**
 * @file ground_conversion.cpp
 */
#include <conversion_tool/ground_conversion.h>
#include <conversion_tool/parser_utils.h>
// TEMPORAL:
#include <geometry_msgs/msg/pose.h>

#include <dlfcn.h>

typedef const rosidl_message_type_support_t * (* get_message_ts_func)();

/**
 * @function GroundConversion
 * @brief Constructor
 */
GroundConversion::GroundConversion() :
rclcpp::Node("ground_conversion",
             rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
{
   //this->declare_parameter("command", rclcpp::PARAMETER_STRING_ARRAY);
   //this->declare_parameter("telemetry", rclcpp::PARAMETER_STRING_ARRAY);
}

/**
 * @function parseComm
 */
bool GroundConversion::parseComm()
{
  std::map<std::string, rclcpp::Parameter> comm_params;

  if (this->get_parameters("communication", comm_params))
  { 
      if(comm_params.find("bridge_port") == comm_params.end())
        return false;
      if(comm_params.find("fsw_port") == comm_params.end())
        return false;
      if(comm_params.find("fsw_ip") == comm_params.end())
        return false;

      own_port_ = comm_params["bridge_port"].as_int();
      fsw_port_ = comm_params["fsw_port"].as_int();      
      fsw_ip_ = comm_params["fsw_ip"].as_string();

      RCLCPP_INFO(this->get_logger(), "Got bridge port: %d fsw port: %d fsw_ip: %s ", own_port_, fsw_port_, fsw_ip_.c_str());
      return true;
  }

  return false;
}

/**
 * @function parseConfigParams
 * @brief Read command and telemetry data
 */
bool GroundConversion::parseConfigParams()
{
  if(!parseComm())
  {
    RCLCPP_ERROR(this->get_logger(), "Parse communication parameters");
    return false;
  }
  
  // Cmd
  rclcpp::Parameter cmd_param;
  std::vector<std::string> cmd_vals;

  if( !this->get_parameter("command", cmd_param) )
  {
    RCLCPP_ERROR(this->get_logger(), "Command parameter not parsed");
    return false;
  }  

  cmd_vals = cmd_param.as_string_array();
  
  for(auto cmd_key : cmd_vals)
  {
     std::map<std::string, rclcpp::Parameter> cmd_params;
     CmdInfo_t ci;

     if (this->get_parameters(cmd_key, cmd_params))
     { 
      if(cmd_params.find("type") == cmd_params.end())
        continue;
      if(cmd_params.find("topic") == cmd_params.end())
        continue;
      if(cmd_params.find("mid") == cmd_params.end())
        continue;

      ci.msg_type = cmd_params["type"].as_string();
      ci.topic = cmd_params["topic"].as_string();
      ci.mid = cmd_params["mid"].as_int() & 0xFFFF;
      if(!extract_type_identifier(ci.msg_type, ci.interface_name, ci.interface_type))
      {
       RCLCPP_ERROR(this->get_logger(), "Error getting interface type and name");
      }
      
      cmd_info_[ci.topic] = ci;
      
      RCLCPP_INFO(this->get_logger(), "*** CMD: Got type: %s, interface name: %s, interface type: %s topic: %s and mid: %02x, %02x", ci.msg_type.c_str(), ci.interface_name.c_str(), ci.interface_type.c_str(), ci.topic.c_str(), (ci.mid >> 8) & 0xFF, ci.mid & 0xFF );

      // Add subscriber
      this->addSubscriber( ci.topic, ci.msg_type );
     }
     
  } // for cmd 

  // Tlm
  rclcpp::Parameter tlm_param;
  std::vector<std::string> tlm_vals;
  
  if( !this->get_parameter("telemetry", tlm_param) )
  {
    RCLCPP_ERROR(this->get_logger(), "telemetry parameter not parsed");
    return false;
  }

  tlm_vals = tlm_param.as_string_array();


  for(auto ti : tlm_vals)
  {
     std::map<std::string, rclcpp::Parameter> tlm_params;
     std::string type_str, topic_str;

     if (this->get_parameters(ti, tlm_params))
     { 
      if(tlm_params.find("type") == tlm_params.end())
        continue;
      if(tlm_params.find("topic") == tlm_params.end())
        continue;

      type_str = tlm_params["type"].as_string();
      topic_str = tlm_params["topic"].as_string();
      RCLCPP_INFO(this->get_logger(), "*** TLM: Got type and topic: %s and %s", type_str.c_str(), topic_str.c_str());

      // Add subscriber
      this->addPublisher(topic_str, type_str);
     }
     
  } // for tlm 

  return true;
}

/**
 * @function initCommunication
 */
bool GroundConversion::initCommunication() 
{
   std::string error_str;
   return bc_.initialize( own_port_, fsw_port_, fsw_ip_, error_str);
}

/**
 * @function subscriberCallback
 */
void GroundConversion::subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, const std::string &_topic_name)
{
   if( cmd_info_.find(_topic_name) == cmd_info_.end() )
  {
     RCLCPP_ERROR(this->get_logger(), "Cmd Info has not topic name stored, not trying to serialize");
     return;
  }
  
  std::string interface_name = cmd_info_[_topic_name].interface_name;
  std::string interface_type = cmd_info_[_topic_name].interface_type;
  
  RCLCPP_INFO( this->get_logger(), " Message type for topic: %s - Interface name: %s , type: %s", 
               _topic_name.c_str(), interface_name.c_str(), interface_type.c_str()); 
  

  auto library = rclcpp::get_typesupport_library(
      interface_name + "/" + interface_type, "rosidl_typesupport_c");
  auto type_support = rclcpp::get_message_typesupport_handle(
      interface_name + "/" + interface_type, "rosidl_typesupport_c", *library);
    
  //uint8_t* data; // = reinterpret_cast<uint8_t *>(&ros_msg);

  RCLCPP_INFO(this->get_logger(), "Get type info");
  std::string error_msg;
  const rosidl_typesupport_introspection_c__MessageMembers * type_info =  
  get_type_info(interface_name, 
                interface_type,
                error_msg);


/////////////////////////////////////////////////////////
rcutils_allocator_t * allocator;
  rcutils_allocator_t default_allocator = rcutils_get_default_allocator();
  RCLCPP_INFO(this->get_logger(), "Allocating message buffer of size %ld bytes",
    type_info->size_of_);
    allocator = &default_allocator;
  // Allocate space to store the binary representation of the message
  uint8_t * data =
    static_cast<uint8_t *>(allocator->allocate(type_info->size_of_, allocator->state));
  if (nullptr == data) {
    RCLCPP_ERROR(this->get_logger(), "Error allocating");
    return;
  }
  // Initialise the message buffer according to the interface type
  type_info->init_function(data, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
  //////////////////////////////////////////////////


  RCLCPP_INFO(this->get_logger(), "Rmw deserialize....");  
  auto ret = rmw_deserialize(&(_msg.get()->get_rcl_serialized_message()), type_support, data);
  
  
    if (ret != RMW_RET_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to deserialize ROS message in C, it seems");
      return;
    }

     RCLCPP_INFO(this->get_logger(), "Beginning of for...");
  for (uint32_t ii = 0; ii < type_info->member_count_; ++ii) {
    // Get the introspection information for this particular member
    const rosidl_typesupport_introspection_c__MessageMember & member_info = type_info->members_[ii];
    // Get a pointer to the member's data in the binary buffer
    uint8_t * member_data = &data[member_info.offset_];
    // Recursively (because some members may be non-primitive types themselves) convert the member
    //yaml_msg[member_info.name_] = dynmsg::c::impl::member_to_yaml(member_info, member_data);
    member_to_yaml(member_info, member_data);
    RCLCPP_INFO(this->get_logger(), "Member[%lu] offset: %d -- Member info name: %s \n", ii, member_info.offset_, member_info.name_);
  } 
   
   
  // Serialize the data
  unsigned char* data_buffer = 0;
  size_t  data_size; // = serialize(data, &buf);
  
  uint16_t mid;
  uint8_t code = 0x01;
  uint16_t seq = 0;
  
  // Hard-code to test
  //if(
  
  //bc_.sendCmdPacket(mid, code, seq, &data_buffer, data_size);
}

void member_to_yaml(
  const rosidl_typesupport_introspection_c__MessageMember & member_info,
  uint8_t * member_data)
  if (member_info.is_array_) {
    //YAML::Node array;
    if (member_info.is_upper_bound_ || member_info.array_size_ == 0) {
      dynamic_array_to_yaml(member_info, member_data, array);
    //} else {
    //  fixed_array_to_yaml(member_info, member_data, array);
    //}

  }

/**
 * @function addSubscriber
 * @brief The signature for the subscription callback function can be one of many options. We normally use
 * the signature with just the shared pointer to the message. Here we use the one with the message info argument
 * added. This is not really needed for our application right now, but I am using it to remember in the future that
 * we can use this additional argument if needed :D.
 */
bool GroundConversion::addSubscriber(const std::string &_topic_name, const std::string &_message_type)
{
 
 auto sub = this->create_generic_subscription(_topic_name, _message_type,
      rclcpp::QoS(1), 
      [this, _topic_name](std::shared_ptr<const rclcpp::SerializedMessage> _msg, const rclcpp::MessageInfo & _mi)
      {
         this->subscriberCallback(_msg, _topic_name);
      } );

  subscribers_[_topic_name] = sub;

  return true;
}


/**
 * @function addPublisher
 */
bool GroundConversion::addPublisher(const std::string &_topic_name, const std::string &_message_type)
{
  auto pub = this->create_generic_publisher(_topic_name, _message_type, rclcpp::QoS(1).transient_local());
  publishers_[_topic_name] = pub;

  return true;
}



