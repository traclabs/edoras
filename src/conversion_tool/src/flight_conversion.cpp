/**
 * @file flight_conversion.cpp
 */
#include <conversion_tool/flight_conversion.h>
#include <conversion_tool/parser_utils.h>
#include <conversion_tool/debug_utils.h>

#include <dlfcn.h>

using namespace std::placeholders;

typedef const rosidl_message_type_support_t * (* get_message_ts_func)();

/**
 * @function FlightConversion
 * @brief Constructor
 */
FlightConversion::FlightConversion() :
rclcpp::Node("flight_conversion",
             rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
{
   tlm_rate_ms_ = 100; // Every 100 ms check for new incoming data (10Hz) - sbn_receiver.py
   scanning_rate_ms_ = 500; // Every 500 ms (0.5s)
}

/**
 * @function parseComm
 */
bool FlightConversion::parseComm()
{
  std::map<std::string, rclcpp::Parameter> comm_params;

  if (this->get_parameters("communication", comm_params))
  {
      if(comm_params.find("peer_ip") == comm_params.end())
        return false;
      if(comm_params.find("peer_port") == comm_params.end())
        return false;
      if(comm_params.find("peer_spacecraft_id") == comm_params.end())
        return false;
      if(comm_params.find("peer_processor_id") == comm_params.end())
        return false;

      if(comm_params.find("udp_receive_port") == comm_params.end())
        return false;
      if(comm_params.find("udp_receive_ip") == comm_params.end())
        return false;
      if(comm_params.find("spacecraft_id") == comm_params.end())
        return false;
      if(comm_params.find("processor_id") == comm_params.end())
        return false;

      peer_ip_ = comm_params["peer_ip"].as_string();
      peer_port_ = comm_params["peer_port"].as_int();
      peer_spacecraft_id_ = comm_params["peer_spacecraft_id"].as_int();
      peer_processor_id_ = comm_params["peer_processor_id"].as_int();
      
      udp_receive_port_ = comm_params["udp_receive_port"].as_int();
      udp_receive_ip_ = comm_params["udp_receive_ip"].as_string();
      spacecraft_id_ = comm_params["spacecraft_id"].as_int();
      processor_id_ = comm_params["processor_id"].as_int();

      RCLCPP_INFO(this->get_logger(), "** ParseComm: \n * peer ip: %s \n * peer port: %d \n * peer processor id: %d \n* peer spacecraft id: %d \n * udp_receive_ip: %s \n * udp_receive_port: %d \n * spacecraft_id: %d \n * processor_id: %d ", peer_ip_.c_str(), peer_port_, peer_processor_id_, peer_spacecraft_id_, udp_receive_ip_.c_str(), udp_receive_port_, spacecraft_id_, processor_id_);
      return true;
  }

  return false;
}

bool FlightConversion::loadTelemetryInfo( const std::vector<std::string> &_tlm_vals)
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
bool FlightConversion::loadCommandInfo( const std::vector<std::string> &_cmd_vals)
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
bool FlightConversion::parseConfigParams()
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
bool FlightConversion::initCommunication() 
{
   std::string error_str;
   
   // Initialize communication
   RCLCPP_INFO(this->get_logger(), "*** Initializing Peer Communication");
   pc_.reset(new PeerCommunication(shared_from_this()));
   if(!pc_->initialize( udp_receive_port_, udp_receive_ip_, spacecraft_id_, processor_id_, error_str))
   {
      RCLCPP_INFO(this->get_logger(), "Failed in initializing the Peer Communication module");
      return false;
   }

   // Start communication timer
   timer_sub_scanning_ = this->create_wall_timer(std::chrono::milliseconds(scanning_rate_ms_), 
                 std::bind(&FlightConversion::subscriptionScanning, this));
      
   timer_comm_tlm_ = this->create_wall_timer(std::chrono::milliseconds(tlm_rate_ms_), std::bind(&FlightConversion::receiveTelemetry, this));   	

 
   // Add peer (fsw)
   RCLCPP_INFO(this->get_logger(), "*** Adding Peer: sc id : %d processor id: %d", peer_spacecraft_id_, peer_processor_id_);
   if(!pc_->addPeer(peer_ip_, peer_port_, peer_spacecraft_id_, peer_processor_id_))
     return false;
 
   return true;
}

/**
 * @function subscriptionScanning
 */
void FlightConversion::subscriptionScanning()
{
   for(auto ti : tlm_info_)
   {
      auto info = this->get_subscriptions_info_by_topic(ti.first);
      if(info.size() > 0)
        pc_->sendAllSubscriptionMsg(ti.second.mid);
      else
        pc_->sendAllUnsubscriptionMsg(ti.second.mid);
   }
}

/**
 * @function receiveTelemetry
 */
void FlightConversion::receiveTelemetry()
{ 
  uint16_t mid;
  std::vector<uint8_t> tlm_header_debug;
  uint8_t* buffer = NULL;  
  bool received_data;
  do
  {
     received_data = pc_->receiveTlmPacket(mid, &buffer);
     if( received_data )
     {
        // Check if this telemetry's mid is one our application cares to hear
        std::string topic_name;
        if( hasMid(mid, topic_name) )
        {  
           RCLCPP_DEBUG(this->get_logger(), "Mid received (%04x) corresponds to topic: %s .", mid, topic_name.c_str());

           // Publish data
           rcutils_uint8_array_t* serialized_array = nullptr;
           serialized_array = make_serialized_array(buffer);
           rclcpp::SerializedMessage serialized_msg(*serialized_array);
           publishers_[topic_name]->publish(serialized_msg);
         
           // Clean up
           free(buffer);
           rmw_ret_t res = rcutils_uint8_array_fini(serialized_array);
           if(res != RCUTILS_RET_OK)
             RCLCPP_ERROR(this->get_logger(), "releasing resources from serialized_array used to publish  tlm!");
     
         } // hasMid
     } // if received_data    

  } while(received_data);

}

/**
 * @function getBufferString
 */
std::string FlightConversion::getBufferString(uint8_t* _buffer, size_t _buffer_size)
{
   if(_buffer == NULL)
     return std::string("");
     
   std::string s = "";
   for(size_t i = 0; i < _buffer_size; i++)
   {   
      char bi[10];
      sprintf(bi, "%02x", *(_buffer + i) );
      s = s +  " " + bi;
      if(i % 8 == 7 )
        s = s + "\n";
   }

   return s;
}

/**
 * @function hasMid
 * @brief Check whether this mid corresponds to a tlm item indicated in the config yaml file 
 */
bool FlightConversion::hasMid(const uint16_t &_mid, std::string &_topic)
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
void FlightConversion::subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, const std::string &_topic_name)
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
  // Send data to cFS
  pc_->send(mid, &data_buffer, data_buffer_size);

}


/**
 * @function addSubscriber
 * @brief The signature for the subscription callback function can be one of many options. We normally use
 * the signature with just the shared pointer to the message. Here we use the one with the message info argument
 * added. This is not really needed for our application right now, but I am using it to remember in the future that
 * we can use this additional argument if needed :D.
 */
bool FlightConversion::addSubscriber(const std::string &_topic_name, const std::string &_message_type)
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
bool FlightConversion::addPublisher(const std::string &_topic_name, const std::string &_message_type)
{
  auto pub = this->create_generic_publisher(_topic_name, _message_type, rclcpp::QoS(1).transient_local());
  publishers_[_topic_name] = pub;

  return true;
}



