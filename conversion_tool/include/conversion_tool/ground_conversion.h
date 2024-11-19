#include <rclcpp/rclcpp.hpp>


class GroundConversion : public rclcpp::Node {

public:

GroundConversion();

bool parseConfigParams();


protected:

bool addPublisher(const std::string &_topic_name, const std::string &_message_type);
bool addSubscriber(const std::string &_topic_name, const std::string &_message_type);

void subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg);

std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericPublisher> > publishers_;
std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericSubscription> > subscribers_;

};