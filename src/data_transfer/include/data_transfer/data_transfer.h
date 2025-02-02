/**
 * @file data_transfer.h
 */
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <edoras_msgs/srv/start_store_data.hpp>
#include <edoras_msgs/srv/stop_store_data.hpp>
#include <edoras_msgs/srv/add_store_data.hpp>
#include <mutex>

struct StoreDataInfo_t {
//  StoreDataInfo_t( const std::string &_msg_type, const std::string &_topic,
 //                  int _write_rate );
                   
  std::string msg_type;
  std::string topic;
  int write_rate;
  std::string store_pkg;
  std::string store_filename;
  rclcpp::TimerBase::SharedPtr timer_save;
  rclcpp::SerializedMessage data;
  std::mutex mux;

};

/**
 * @class DataTransfer
 */
class DataTransfer : public rclcpp::Node {

public:

  DataTransfer();
  bool initialize();
  bool parseConfigParams();

protected:

  bool loadWriteData( const std::vector<std::string> &_data_vals);
  bool loadReadData( const std::vector<std::string> &_data_vals);
  
  void add_store(const std::shared_ptr<edoras_msgs::srv::AddStoreData::Request> _req,
                 std::shared_ptr<edoras_msgs::srv::AddStoreData::Response> _res);
  void start_store(const std::shared_ptr<edoras_msgs::srv::StartStoreData::Request> _req,
                 std::shared_ptr<edoras_msgs::srv::StartStoreData::Response> _res);
  void stop_store(const std::shared_ptr<edoras_msgs::srv::StopStoreData::Request> _req,
                 std::shared_ptr<edoras_msgs::srv::StopStoreData::Response> _res);  
  
  void writeData(const std::string &_topic_name);
  void readData(const std::string &_topic_name);  
  bool addPublisher(const std::string &_topic_name, const std::string &_message_type);
  bool addSubscriber(const std::string &_topic_name, const std::string &_message_type);

  void subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, 
                          const std::string &_topic_name);
                                
  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericPublisher> > publishers_;
  std::unordered_map< std::string, std::shared_ptr<rclcpp::GenericSubscription> > subscribers_;

  // Communication
  rclcpp::TimerBase::SharedPtr timer_comm_tlm_;
  int tlm_rate_ms_;
  
  //
  std::map<std::string, std::shared_ptr<StoreDataInfo_t> > write_info_;
 std::map<std::string, std::shared_ptr<StoreDataInfo_t> > read_info_;
   
  // Helper
  rclcpp::Service<edoras_msgs::srv::AddStoreData>::SharedPtr srv_add_store_; 
  rclcpp::Service<edoras_msgs::srv::StartStoreData>::SharedPtr srv_start_store_;
  rclcpp::Service<edoras_msgs::srv::StopStoreData>::SharedPtr srv_stop_store_;  
};
