/**
 * @file ground_conversion.h
 */
#include <rclcpp/rclcpp.hpp>
#include <conversion_tool/types.h>
#include <conversion_tool/basic_communication.h>

struct CmdInfo_t {
  std::string msg_type;
  std::string topic;
  uint16_t mid;
  std::string interface_name;
  std::string interface_type;
  std::shared_ptr<rcpputils::SharedLibrary> library; 
  const TypeSupport_t* type_support;
  const TypeInfo_t* type_info;
};

/**
 * @class GroundConversion
 */
class GroundConversion : public rclcpp::Node {

public:

  GroundConversion();
  bool parseConfigParams();
  bool initCommunication();

protected:

  bool parseComm();
  void receiveTelemetry();
  bool loadCommandInfo( const std::vector<std::string> &_cmd_vals);
  bool addPublisher(const std::string &_topic_name, const std::string &_message_type);
  bool addSubscriber(const std::string &_topic_name, const std::string &_message_type);

  void subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, const std::string &_topic_name);

  void member_to_yaml(const rosidl_typesupport_introspection_c__MessageMember & member_info, uint8_t * member_data);

  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericPublisher> > publishers_;
  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericSubscription> > subscribers_;

  // Params
  int own_port_;
  int fsw_port_;
  std::string fsw_ip_;

  // Communication
  BasicCommunication bc_; 
  rclcpp::TimerBase::SharedPtr timer_comm_tlm_;
  
  //
  std::map<std::string, CmdInfo_t> cmd_info_;

};
