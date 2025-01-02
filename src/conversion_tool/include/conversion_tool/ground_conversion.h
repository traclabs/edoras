/**
 * @file ground_conversion.h
 */
#include <rclcpp/rclcpp.hpp>
#include <conversion_tool/types.h>
#include <conversion_tool/basic_communication.h>
#include <std_srvs/srv/set_bool.hpp>

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

struct TlmInfo_t {
  std::string msg_type;
  std::string topic;
  uint16_t mid;

};

/**
 * @class GroundConversion
 */
class GroundConversion : public rclcpp::Node {

public:

  GroundConversion();
  bool parseConfigParams();
  bool initCommunication();
  bool enableTOLabOutputCmd(bool _enable);

protected:

  bool parseComm();
  void receiveTelemetry();
  bool loadCommandInfo( const std::vector<std::string> &_cmd_vals);
  bool loadTelemetryInfo( const std::vector<std::string> &_tlm_vals);
  
  bool hasMid(const uint16_t &_mid, std::string &_topic);
  bool addPublisher(const std::string &_topic_name, const std::string &_message_type);
  bool addSubscriber(const std::string &_topic_name, const std::string &_message_type);

  void subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, const std::string &_topic_name);
  void to_lab_enable_output_cmd(const std::shared_ptr<std_srvs::srv::SetBool::Request> _req,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> _res);

  void member_to_yaml(const rosidl_typesupport_introspection_c__MessageMember & member_info, uint8_t * member_data);

  // Helper functions
  std::string getBufferString(uint8_t* _buffer, size_t _buffer_size);

  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericPublisher> > publishers_;
  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericSubscription> > subscribers_;

  // Params
  int own_port_;
  int fsw_port_;
  std::string fsw_ip_;
  std::string telemetry_ip_;

  // For Phase I easiness
  std::string bridge_ip_;

  // Communication
  BasicCommunication bc_; 
  rclcpp::TimerBase::SharedPtr timer_comm_tlm_;
  
  //
  std::map<std::string, CmdInfo_t> cmd_info_;
  std::map<std::string, TlmInfo_t> tlm_info_;
  
  // Helper to_lab_enable_output_cmd
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_to_lab_; 

};
