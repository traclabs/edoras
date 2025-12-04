
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
  
bool sendMessage( const sensor_msgs::msg::JointState &_js )
{    
    unsigned char* buf     = 0;
    size_t         bufSize = serializeJointState(_js, &buf);
    
    sendto(sockfd_, buf, bufSize, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));
    return true;
  }
  
  
bool receiveMessage(sensor_msgs::msg::JointState &_js)
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


