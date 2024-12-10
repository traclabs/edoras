/**
 * @file ground_conversion.h
 */
#include <rclcpp/rclcpp.hpp>
#include <conversion_tool/basic_communication.h>

struct CmdInfo_t {
  std::string msg_type;
  std::string topic;
  uint16_t mid;
  std::string interface_name;
  std::string interface_type;
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
  bool addPublisher(const std::string &_topic_name, const std::string &_message_type);
  bool addSubscriber(const std::string &_topic_name, const std::string &_message_type);

  void subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, const std::string &_topic_name);

  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericPublisher> > publishers_;
  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericSubscription> > subscribers_;

  // Params
  int own_port_;
  int fsw_port_;
  std::string fsw_ip_;

  // Communication
  BasicCommunication bc_; 

  //
  std::map<std::string, CmdInfo_t> cmd_info_;

};
