
/**
 * @file serialize_arm_manual.cpp
 */
#include <edoras_demos/serialize_arm_manual.h>
 
SerializeArmManual::SerializeArmManual()
{} 
 
bool SerializeArmManual::initializeComm( const int &_own_port, 
                                         const int &_other_port, 
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
    own_address_.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
    own_address_.sin_port = htons(_own_port);


    // Fill cFS information
    other_address_.sin_family = AF_INET;
    other_address_.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
    other_address_.sin_port = htons(_other_port);
  
    // Bind the socket
    int res = bind(sockfd_, (const struct sockaddr*)&own_address_, sizeof(own_address_));
    if( res < 0 )
    {
       _error_msg = "Bind failed";
       return false;
    }

    return true;
}  
  
bool SerializeArmManual::sendMessage( sensor_msgs::msg::JointState* _js )
{    
    unsigned char* buf     = 0;
    size_t         bufSize = serialize(_js, &buf);
    
    int res = sendto(sockfd_, buf, bufSize, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));
    
    // Clean up
    free(buf);
    
    return (res > 0);
}
  
  
bool SerializeArmManual::receiveMessage(geometry_msgs::msg::Pose &_ps)
{
     ssize_t buffer_rcvd_size; 
     const int MAXLINE = 1024;
     uint8_t buffer_rcvd[MAXLINE];

     // Receive............
    buffer_rcvd_size = recvfrom(sockfd_, (uint8_t*) buffer_rcvd, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(buffer_rcvd_size > 0)
    {
      _ps = deserialize(buffer_rcvd, (size_t) buffer_rcvd_size, 0);
      return true;  
    }

  return false;
}
/////////////////////////////

size_t SerializeArmManual::serialize(sensor_msgs::msg::JointState *_js, uint8_t** _buf)
{
   if (_js->name.size() == 0 || _js->position.size() == 0) 
     return 0;

   size_t num_joints = _js->position.size();
   size_t data_size = num_joints * sizeof(double) + sizeof(int32_t) + sizeof(uint32_t); // joints + sec + nanosec
   
   *_buf = static_cast<uint8_t *> (malloc(data_size));
   if (*_buf)
   {
      double data[num_joints];
      for(int i = 0; i < num_joints; i++)
          data[i] = _js->position[i];

      size_t offset = 0;
      for(int i = 0; i < num_joints; i++)
      {
        memcpy(*_buf + offset, &data[i], sizeof(double) );
        offset += sizeof(double);
      }

     int32_t sec = _js->header.stamp.sec;
     uint32_t nanosec = _js->header.stamp.nanosec;
     
     memcpy(*_buf + offset, &sec, sizeof(int32_t));
     offset += sizeof(int32_t);
     memcpy(*_buf + offset, &nanosec, sizeof(uint32_t));
     offset += sizeof(uint32_t);
     printf("Should be sending sec: %d nanosec: %d \n", sec, nanosec);
     return data_size;
           
   } else
     return 0;
}

geometry_msgs::msg::Pose SerializeArmManual::deserialize(const uint8_t* _buf, const size_t bufSize, size_t start_offset)
{
 geometry_msgs::msg::Pose pose;

  size_t offset = 0;
  memcpy(&pose.position.x, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&pose.position.y, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&pose.position.z, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&pose.orientation.x, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&pose.orientation.y, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&pose.orientation.z, _buf + offset, sizeof(double)); offset += sizeof(double);
  memcpy(&pose.orientation.w, _buf + offset, sizeof(double)); offset += sizeof(double);  

 return pose;
}

