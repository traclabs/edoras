
/**
 * @file serialize_manual.cpp
 */
#include <edoras_demos/serialize_manual.h>
 
SerializeManual::SerializeManual()
{} 
 
bool SerializeManual::initializeComm( const int &_own_port, const int &_other_port, std::string &_error_msg)
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
  
bool SerializeManual::sendMessage( sensor_msgs::msg::JointState* _js )
{    
    unsigned char* buf     = 0;
    size_t         bufSize = serialize(_js, &buf);
    
    int res = sendto(sockfd_, buf, bufSize, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));
    
    if(res < 0)
      return false;
    
    return true;
  }
  
  
bool SerializeManual::receiveMessage(sensor_msgs::msg::JointState &_js)
  {
  /*
     int n; 
     // Receive............
    n = recvfrom(sockfd, (unsigned char*) buffer, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    if(n > 0)
    {
      struct Data* data2  = deserialize(buffer, n, 0);
      printf("\t Data 2: Received message with bytes: %d . Age: %d . First name: %s . Last name: %s \n", n, data2->age, data2->first_name, data2->last_name);
    }
*/
  return true;  
}
/////////////////////////////

size_t SerializeManual::serialize(sensor_msgs::msg::JointState *_js, unsigned char** buf)
{

   if (_js->name.size() == 0 || _js->position.size() == 0) 
     return 0;

   //const size_t name_size   = _js.name.size();
   //sizeof(size_t) + name_size*sizeof(double);
   const size_t position_size = _js->position.size();
   const size_t data_size =  sizeof(size_t) + position_size*sizeof(double);
   
   *buf = (unsigned char*)calloc(1, data_size);

   if (*buf)
   {
      size_t offset = 0;
  
      // age
      //unsigned int tmp = htonl(data->age);
      //memcpy(*buf + offset, &tmp, sizeof(tmp));                offset += sizeof(tmp);

      // length of position and positions themselves
      size_t tmp_l = htonl(position_size);
      memcpy(*buf + offset, &tmp_l, sizeof(tmp_l));            offset += sizeof(tmp_l);
      for(int i = 0; i < position_size; ++i)
      {
        memcpy(*buf + offset, &_js->position[i], sizeof(double) );        
        offset += sizeof(double);
      }
      
   }

   return data_size;

}

sensor_msgs::msg::JointState SerializeManual::deserialize(const unsigned char* buf, const size_t bufSize, size_t start_offset)
{

}

