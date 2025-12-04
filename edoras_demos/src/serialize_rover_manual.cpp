
/**
 * @file serialize_rover_manual.cpp
 */
#include <edoras_demos/serialize_rover_manual.h>
 
SerializeRoverManual::SerializeRoverManual()
{} 
 
bool SerializeRoverManual::initializeComm( const int &_own_port, 
                                           const int &_fsw_port,
                                           const std::string &_robot_ip, 
                                           const std::string &_fsw_ip, 
                                           std::string &_error_msg)
{
    // Create socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd_ < 0)
    {
       _error_msg = "Socket creationg failed";
       return false;
    }
  
    memset(&own_address_, 0, sizeof(own_address_));
    memset(&other_address_, 0, sizeof(other_address_));
  
    // Fill server information
    own_address_.sin_family = AF_INET;
    own_address_.sin_addr.s_addr = inet_addr(_robot_ip.c_str()); // "127.0.0.1" //INADDR_ANY;
    own_address_.sin_port = htons(_own_port);


    // Fill cFS information
    other_address_.sin_family = AF_INET;
    other_address_.sin_addr.s_addr = inet_addr(_fsw_ip.c_str()); // "127.0.0.1" //INADDR_ANY;
    other_address_.sin_port = htons(_fsw_port);
  
    // Bind the socket
    int res = bind(sockfd_, (const struct sockaddr*)&own_address_, sizeof(own_address_));
    if( res < 0 )
    {
       _error_msg = "Bind failed";
       return false;
    }

    return true;
}  

/**
 * @function sendMessage
 */  
bool SerializeRoverManual::sendMessage( geometry_msgs::msg::PoseStamped* _ps )
{    
    uint8_t* buf     = NULL;
    size_t         bufSize = serialize(_ps, &buf);
    int res = sendto(sockfd_, buf, bufSize, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));
  
    // Clean up
    free(buf);
    
    return (res > 0);
}
  
  
bool SerializeRoverManual::receiveMessage(geometry_msgs::msg::Twist &_tm)
{
     ssize_t buffer_rcvd_size; 
     const int MAXLINE = 1024;
     uint8_t buffer_rcvd[MAXLINE];

     // Receive............
    buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(buffer_rcvd_size > 0)
    {
      _tm = deserialize(buffer_rcvd, (size_t) buffer_rcvd_size, 0);
      return true;  
    }

  return false;
}
/////////////////////////////

size_t SerializeRoverManual::serialize(geometry_msgs::msg::PoseStamped *_ps, uint8_t** _buf)
{
   // 7 : 3 (position=xyz) + 4 (orientation=xyzw) float64 == double
   size_t data_size =  7*sizeof(double) + sizeof(int32_t) + sizeof(uint32_t);
   
   *_buf = static_cast<uint8_t *> (malloc(data_size));
   if (*_buf)
   {
      double data[7] = { _ps->pose.position.x, _ps->pose.position.y, _ps->pose.position.z, 
                       _ps->pose.orientation.x, _ps->pose.orientation.y, 
                       _ps->pose.orientation.z, _ps->pose.orientation.w };

      size_t offset = 0;
      for(int i = 0; i < 7; ++i)
      {
        memcpy(*_buf + offset, &data[i], sizeof(double) );
        offset += sizeof(double);
      }

      int32_t sec = _ps->header.stamp.sec;
      uint32_t nanosec = _ps->header.stamp.nanosec;
      
      memcpy(*_buf + offset, &sec, sizeof(int32_t) );
      offset += sizeof(int32_t);
      memcpy(*_buf + offset, &nanosec, sizeof(uint32_t) );
      offset += sizeof(uint32_t);      

     return data_size;
           
   } else
     return 0;
}

geometry_msgs::msg::Twist SerializeRoverManual::deserialize(const uint8_t* _buf, const size_t bufSize, size_t start_offset)
{
 geometry_msgs::msg::Twist twist;

  size_t offset = 0;
  memcpy(&twist.linear.x, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&twist.linear.y, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&twist.linear.z, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&twist.angular.x, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&twist.angular.y, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&twist.angular.z, _buf + offset, sizeof(double)); offset += sizeof(double);        

 return twist;
}

