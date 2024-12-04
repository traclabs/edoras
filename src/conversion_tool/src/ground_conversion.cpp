#include <conversion_tool/ground_conversion.h>



GroundConversion::GroundConversion() :
rclcpp::Node("ground_conversion",
             rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
{
   //this->declare_parameter("command", rclcpp::PARAMETER_STRING_ARRAY);
   //this->declare_parameter("telemetry", rclcpp::PARAMETER_STRING_ARRAY);

}

bool GroundConversion::parseConfigParams()
{
  // Cmd
  rclcpp::Parameter cmd_param;
  std::vector<std::string> cmd_vals;

  if( !this->get_parameter("command", cmd_param) )
  {
    RCLCPP_ERROR(this->get_logger(), "Command parameter not parsed");
    return false;
  }  

  cmd_vals = cmd_param.as_string_array();
  
  for(auto ci : cmd_vals)
  {
     std::map<std::string, rclcpp::Parameter> cmd_params;
     std::string type_str, topic_str;

     if (this->get_parameters(ci, cmd_params))
     { 
      if(cmd_params.find("type") == cmd_params.end())
        continue;
      if(cmd_params.find("topic") == cmd_params.end())
        continue;

      type_str = cmd_params["type"].as_string();
      topic_str = cmd_params["topic"].as_string();
      RCLCPP_INFO(this->get_logger(), "Got type and topic: %s and %s", type_str.c_str(), topic_str.c_str());

      // Add subscriber
      this->addSubscriber(topic_str, type_str);
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
      RCLCPP_INFO(this->get_logger(), "Got type and topic: %s and %s", type_str.c_str(), topic_str.c_str());

      // Add subscriber
      this->addPublisher(topic_str, type_str);
     }
     
  } // for tlm 

  return true;
}

void GroundConversion::subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg)
{
 RCLCPP_INFO( this->get_logger(), "Subscriber received a message! \n");
}

bool GroundConversion::addSubscriber(const std::string &_topic_name, const std::string &_message_type)
{
 auto sub = this->create_generic_subscription(_topic_name, _message_type,
      rclcpp::QoS(1), 
      std::bind(&GroundConversion::subscriberCallback, this, std::placeholders::_1) );
  subscribers_[_topic_name] = sub;

  return true;
}

bool GroundConversion::addPublisher(const std::string &_topic_name, const std::string &_message_type)
{
  auto pub = this->create_generic_publisher(_topic_name, _message_type, rclcpp::QoS(1).transient_local());
  publishers_[_topic_name] = pub;

  return true;
}



