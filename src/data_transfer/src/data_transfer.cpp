/**
 * @file data_transfer.cpp
 */
#include <data_transfer/data_transfer.h>
//#include <conversion_tool/parser_utils.h>
//#include <conversion_tool/debug_utils.h>

using namespace std::placeholders;

/*
StoreDataInfo_t::StoreDataInfo_t( const std::string &_msg_type, 
                                  const std::string &_topic,
                                  int _write_rate )
{
   msg_type = _msg_type;
   topic = _topic;
   write_rate = _write_rate;
}    */                              

/**
 * @function DataTransfer
 * @brief Constructor
 */
DataTransfer::DataTransfer() :
rclcpp::Node("data_transfer",
             rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
{
   tlm_rate_ms_ = 100; // Every 100 ms check for new incoming data (10Hz)
}

/**
 * @function initialize
 */
bool DataTransfer::initialize() 
{
   // Read parameters
   if (!parseConfigParams())
     return false;
   
   // Set up services
   srv_stop_store_ = this->create_service<edoras_msgs::srv::StopStoreData>("stop_store", 
                     std::bind(&DataTransfer::stop_store, this, _1, _2));
   srv_start_store_ = this->create_service<edoras_msgs::srv::StartStoreData>("start_store", 
                     std::bind(&DataTransfer::start_store, this, _1, _2));
   srv_add_store_ = this->create_service<edoras_msgs::srv::AddStoreData>("add_store", 
                     std::bind(&DataTransfer::add_store, this, _1, _2));
                                                
   return true;
}

/**
 * @function parseConfigParams
 * @brief Read command and telemetry data
 */
bool DataTransfer::parseConfigParams()
{
  // Parse data params
  rclcpp::Parameter store_data_param;
  std::vector<std::string> data_vals;

  if( !this->get_parameter("store_data", store_data_param) )
  {
    RCLCPP_ERROR(this->get_logger(), "'store_data' parameter not parsed");
    return false;
  }  

  data_vals = store_data_param.as_string_array();
  return loadStoreData(data_vals);
}

/**
 * @function loadStoreData
 */
bool DataTransfer::loadStoreData( const std::vector<std::string> &_data_vals)
{
  for(auto data_key : _data_vals)
  {
     std::map<std::string, rclcpp::Parameter> data_params;
     StoreDataInfo_t di;

     if (this->get_parameters(data_key, data_params))
     {
      if(data_params.find("type") == data_params.end())
        continue;
      if(data_params.find("topic") == data_params.end())
        continue;
      if(data_params.find("rate") == data_params.end())
        continue;
      
      std::shared_ptr<StoreDataInfo_t> di = std::make_shared<StoreDataInfo_t>();
      
      std::string msg_type = data_params["type"].as_string();
      std::string topic = data_params["topic"].as_string();
      int write_rate = data_params["rate"].as_int();
      di->msg_type = msg_type;
      di->topic = topic;
      di->write_rate = write_rate;
      di->timer_save = this->create_wall_timer(std::chrono::milliseconds(write_rate),
                      [this, topic]()
                      {
                         this->writeData(topic);
                      } );
       
      data_info_[di->topic] = di;       
      
      RCLCPP_INFO(this->get_logger(), "*** StoreData: Got type: %s, topic: %s and write-rate: %d", 
                  di->msg_type.c_str(), di->topic.c_str(), 
                  di->write_rate);

      // Add subscriber
      this->addSubscriber(di->topic, di->msg_type);
     }
     
  } // for tlm 

  return true;
}

/**
 * @function stop_store
 */
void DataTransfer::stop_store(const std::shared_ptr<edoras_msgs::srv::StopStoreData::Request> _req,
                              std::shared_ptr<edoras_msgs::srv::StopStoreData::Response> _res)
{ 
  // Get DataStore object
  if( data_info_.find(_req->topic) == data_info_.end())
  {
    _res->result = false;
    _res->info = "Topic name not found in map";
    return;
  }
    
  // Stop timer
  try {
    data_info_[_req->topic]->timer_save->cancel();
  } catch(const std::runtime_error& error) {
    _res->result = false;
    _res->info = "Failure canceling the timer for topic";
  }

  _res->result = true;
}

/**
 * @function start_store
 */
void DataTransfer::start_store(const std::shared_ptr<edoras_msgs::srv::StartStoreData::Request> _req,
                               std::shared_ptr<edoras_msgs::srv::StartStoreData::Response> _res)
{ 
  // Get DataStore object
  if( data_info_.find(_req->topic) == data_info_.end())
  {  
    _res->result = false;
    _res->info = "Topic name not found in map";
    return;
  }  
  // Start timer
  try {
    data_info_[_req->topic]->timer_save->reset();
  } catch(const std::runtime_error& error) {
    _res->result = false;
    _res->info = "Failure reseting the timer for topic";
  }

  _res->result = true;
}

/**
 * @function add_store
 */
void DataTransfer::add_store(const std::shared_ptr<edoras_msgs::srv::AddStoreData::Request> _req,
                             std::shared_ptr<edoras_msgs::srv::AddStoreData::Response> _res)
{ 
  // Check DataStore object does not exist already
  if( data_info_.find(_req->topic) != data_info_.end())
  {
    _res->result = false;
    _res->info = "Topic name is already in the map";
    return;
  }
   
  // Start timer
  try {
    data_info_[_req->topic]->timer_save->reset();
  } catch(const std::runtime_error& error) {
    _res->result = false;
    _res->info = "Failure reseting the timer for topic";
  }

}  


/**
 * @function receiveTelemetry
 */
void DataTransfer::writeData(const std::string &_topic_name)
{ 
  rclcpp::SerializedMessage msg;

  // Get latest serialized msg data from message
  //data_info_[_topic_name].mux.lock();
  msg = data_info_[_topic_name]->data;
  //data_info_[_topic_name].mux.unlock();
  
  // Convert it to serialized low-level data
  uint8_t* data_serialized = nullptr;
  
  // Fill it with latest data
  size_t  data_buffer_size;
  std::string error_msg;  
  size_t test_length, test_capacity;

  //data_serialized = from_rcutils_uint_array_to_uint_buffer(&(msg.get_rcl_serialized_message()), 
  //                  data_buffer_size, test_length, test_capacity);

  // Store in file

  // Cleanup data pointer  
  free(data_serialized);
}

/**
 * @function subscriberCallback
 * @brief Subscribes to ROS topic
 */
void DataTransfer::subscriberCallback(const std::shared_ptr<const rclcpp::SerializedMessage> _msg, 
                                      const std::string &_topic_name)
{
  if( data_info_.find(_topic_name) == data_info_.end() )
  {
     RCLCPP_ERROR(this->get_logger(), " DataInfo has not topic name stored, not trying to serialize");
     return;
  }

  data_info_[_topic_name]->data = *_msg;
}


/**
 * @function addSubscriber
 * @brief The signature for the subscription callback function can be one of many options. We normally use
 * the signature with just the shared pointer to the message. Here we use the one with the message info argument
 * added. This is not really needed for our application right now, but I am using it to remember in the future that
 * we can use this additional argument if needed :D.
 */
bool DataTransfer::addSubscriber(const std::string &_topic_name, const std::string &_message_type)
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
bool DataTransfer::addPublisher(const std::string &_topic_name, const std::string &_message_type)
{
  auto pub = this->create_generic_publisher(_topic_name, _message_type, rclcpp::QoS(1).transient_local());
  publishers_[_topic_name] = pub;

  return true;
}



