/**
 * @file ground_conversion.cpp
 */
#include <conversion_tool/ground_conversion.h>
#include <conversion_tool/parser_utils.h>
#include <conversion_tool/debug_utils.h>

#include <dlfcn.h>

using namespace std::placeholders;

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

bool GroundConversion::loadTelemetryInfo( const std::vector<std::string> &_tlm_vals)
{
  for(auto tlm_key : _tlm_vals)
  {
     std::map<std::string, rclcpp::Parameter> tlm_params;
     TlmInfo_t ti;

     if (this->get_parameters(tlm_key, tlm_params))
     { 
      if(tlm_params.find("type") == tlm_params.end())
        continue;
      if(tlm_params.find("topic") == tlm_params.end())
        continue;
      if(tlm_params.find("mid") == tlm_params.end())
        continue;
      ti.msg_type = tlm_params["type"].as_string();
      ti.topic = tlm_params["topic"].as_string();
      ti.mid = tlm_params["mid"].as_int() & 0xFFFF;
      
      tlm_info_[ti.topic] = ti;       
      
      RCLCPP_INFO(this->get_logger(), "*** TLM: Got type: %s, topic: %s and mid: %02x %02x", ti.msg_type.c_str(), ti.topic.c_str(), 
                  (ti.mid >> 8) & 0xFF, ti.mid & 0xFF);

      // Add subscriber
      this->addPublisher(ti.topic, ti.msg_type);
     }
     
  } // for tlm 

  return true;
}

/**
 * @file loadCommandInfo
 */
bool GroundConversion::loadCommandInfo( const std::vector<std::string> &_cmd_vals)
{  
  for(auto cmd_key : _cmd_vals)
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
      
      // Load library
      ci.library = rclcpp::get_typesupport_library(
                   ci.interface_name + "/" + ci.interface_type, 
                   "rosidl_typesupport_c");
      ci.type_support = rclcpp::get_message_typesupport_handle(
                        ci.interface_name + "/" + ci.interface_type, 
                        "rosidl_typesupport_c", *ci.library);
      std::string error_msg;
      ci.type_info = get_type_info(ci.interface_name, 
                     ci.interface_type,
                     error_msg);
                          
      cmd_info_[ci.topic] = ci;
      
      RCLCPP_INFO(this->get_logger(), "*** CMD: Got type: %s, interface name: %s, interface type: %s topic: %s and mid: %02x, %02x", ci.msg_type.c_str(), ci.interface_name.c_str(), ci.interface_type.c_str(), ci.topic.c_str(), (ci.mid >> 8) & 0xFF, ci.mid & 0xFF );

      // Add subscriber
      this->addSubscriber( ci.topic, ci.msg_type );
     }
     
  } // for cmd 

  return true;
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
  loadCommandInfo(cmd_vals);

  // Tlm
  rclcpp::Parameter tlm_param;
  std::vector<std::string> tlm_vals;
  
  if( !this->get_parameter("telemetry", tlm_param) )
  {
    RCLCPP_ERROR(this->get_logger(), "telemetry parameter not parsed");
    return false;
  }

  tlm_vals = tlm_param.as_string_array();
  loadTelemetryInfo(tlm_vals);

  return true;
}

/**
 * @function initCommunication
 */
bool GroundConversion::initCommunication() 
{
   std::string error_str;
   
   // Initialize communication
   RCLCPP_INFO(this->get_logger(), "*** Initializing Basic Communication");
   if(!bc_.initialize( own_port_, fsw_port_, fsw_ip_, error_str))
   {
      RCLCPP_INFO(this->get_logger(), "Failed in initializing the Basic Communication module");
      return false;
   }
   
   // Set up service
   srv_to_lab_ = this->create_service<std_srvs::srv::SetBool>("to_lab_enable_output_cmd", std::bind(&GroundConversion::to_lab_enable_output_cmd, this, _1, _2));
   
   // Start communication timer
   //timer_comm_tlm_ = this->create_wall_timer(std::chrono::milliseconds(1000), []() -> void { receiveTelemetry(); });
   timer_comm_tlm_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GroundConversion::receiveTelemetry, this));   	
   
   return true;
}

/**
 * @function to_lab_enable_output_cmd
 */
void GroundConversion::to_lab_enable_output_cmd(const std::shared_ptr<std_srvs::srv::SetBool::Request> _req,
                                                std::shared_ptr<std_srvs::srv::SetBool::Response> _res)
{ 
  bool enable = true;
  _res->success = enableTOLabOutputCmd(enable);
}

/**
 * @function enableTOLabOutputCmd
 * @brief TODO: Enable/disable. For now, we are only enabling
 */
bool GroundConversion::enableTOLabOutputCmd(bool _enable)
{  
  // 1. Create command to enable TO LAB Output cmd
  size_t  data_buffer_size = 16;
  
  char dest_ip[data_buffer_size];
  strcpy(dest_ip, fsw_ip_.c_str());
  uint8_t* data_buffer = (uint8_t*)( &dest_ip[0] );
  
  // 2. Sending command to activate telemetry to be sent back
  // got this info from juicer_config/util/cfe_plugin_config.yaml
  uint16_t mid = 0x1880; 
  uint8_t code = 0x06;
  uint16_t seq = 0;
  
  // Send data to cFS
  RCLCPP_INFO(this->get_logger(), "Send TO LAB Enable output cmd, mid: %02x . Buffer size: %lu", mid, data_buffer_size);
  return bc_.sendCmdPacket(mid, code, seq, &data_buffer, data_buffer_size);
}

/**
 * @function receiveTelemetry
 */
void GroundConversion::receiveTelemetry()
{ 
  size_t buffer_size;
  uint16_t mid;
  std::vector<uint8_t> tlm_header_debug;
  if( bc_.receiveTlmPacket(buffer_size, mid, tlm_header_debug))
  {   
     // Check if this telemetry's mid is one our application cares to hear
     std::string topic_name;
     if( hasMid(mid, topic_name) )
     {  
        RCLCPP_INFO(this->get_logger(), "Mid received (%04x) corresponds to topic: %s ", mid, topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "TLm Header received: %02x %02x %02x %02x %02x %02x %02x %02x ", 
         tlm_header_debug[0], tlm_header_debug[1], tlm_header_debug[2], tlm_header_debug[3], 
         tlm_header_debug[4], tlm_header_debug[5], tlm_header_debug[6], tlm_header_debug[7]);
         
         // Publish data
         //publishers_[topic_name]-> ;
     }   
  }
}

/**
 * @function hasMid
 * @brief Check whether this mid corresponds to a tlm item indicated in the config yaml file 
 */
bool GroundConversion::hasMid(const uint16_t &_mid, std::string &_topic)
{
   for(auto ti : tlm_info_)
   {
      if(ti.second.mid == _mid)
      {
         _topic = ti.second.topic;
         return true;
      }
   }
   
   return false;
} 

/**
 * @function subscriberCallback
 * @brief Subscribes to ROS topic corresponding to a command to be sent to cFS
 */
void GroundConversion::subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, const std::string &_topic_name)
{
  if( cmd_info_.find(_topic_name) == cmd_info_.end() )
  {
     RCLCPP_ERROR(this->get_logger(), "Cmd Info has not topic name stored, not trying to serialize");
     return;
  }

  // Deserialize from SerializedMessage to TypeSupport
  uint8_t* data_buffer;
  size_t  data_buffer_size;
  std::string error_msg;
  
  size_t test_length, test_capacity;

  data_buffer = from_rcutils_uint_array_to_uint_buffer(&(_msg.get()->get_rcl_serialized_message()), data_buffer_size, test_length, test_capacity);

  //debug_parse_message(data_buffer, cmd_info_[_topic_name].type_info);
   
  uint16_t mid = cmd_info_[_topic_name].mid;
  uint8_t code = 0x01;
  uint16_t seq = 0;
  
  // Send data to cFS
  RCLCPP_INFO(this->get_logger(), "Send cmd packet, mid: %02x . Buffer size: %lu", mid, data_buffer_size);
  bc_.sendCmdPacket(mid, code, seq, &data_buffer, data_buffer_size);  
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



