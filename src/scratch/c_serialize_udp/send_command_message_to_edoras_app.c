#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <assert.h>
#include <stdio.h>

#include "serialize_library.h"

#define OWN_PORT 1235
#define OTHER_PORT 1234
#define MAXLINE 1024

size_t serializeWithCommandHeader(const struct Data* data, unsigned char** buf_with_header)
{
   // Serialize the data
   unsigned char* buf = 0;
   size_t  bufSize = serialize(data, &buf);
   size_t headerSize = 8;

   // Add on top
   // 2: stream_id: 0x1827 (GATEWAY_APP_CMD_MID) 
   // 2: sequence: 
   // 2: length
   // 1: code
   // 1: checksum
   unsigned short seq;
   unsigned short length = (unsigned short)(bufSize + headerSize) - 7;
   unsigned char  header[8];
   printf("Serializing data. Length: %d \n", length);

   header[0] = 0x18;
   header[1] = 0x27;
   header[2] = (seq >> 8) & 0xFF;
   header[3] = seq & 0xFF;
   header[4] = (length >> 8) & 0xFF;
   header[5] = length & 0xFF;
   header[6] = 0x01;
   header[7] = 0;
   printf("Printing header values \n");
    for (int i = 0; i < 8; i++) {
        printf("%02x ", header[i]);
    } printf("\n");

   // Add header bytes on top of the serialized data
   *buf_with_header = calloc(1, bufSize + headerSize);
   printf("* Header size: %d size of header: %d \n", headerSize, sizeof(header));
   size_t offset = 0;
  
   memcpy(*buf_with_header + offset, &header, sizeof(header));  offset += sizeof(header);
   memcpy(*buf_with_header + offset, buf, bufSize);
   
   
   return length;
} 

///////////////
struct Data* deserializeWithCommandHeader(const unsigned char* bufHeader, const size_t bufHeaderSize)
{
  // You know the first 3 bytes are the header
  size_t offset = 0;
  unsigned char header[8];
  memcpy(&header, bufHeader + offset, sizeof(header));
  printf("Deserializing: header: %02x, %02x, %02x \n", header[0], header[1], header[2]);
  offset = offset + sizeof(header);
  struct Data* data = deserialize(bufHeader, bufHeaderSize - sizeof(header), offset);
  return data;
}

/////////////////////////////////////////////
int main()
{
  int sockfd;
  char buffer[MAXLINE];
  struct sockaddr_in own_address_;
  struct sockaddr_in other_address_;
  
  // Create socket
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sockfd < 0)
  {
   perror("Socket creationg failed \n");
   exit(EXIT_FAILURE);
  }
  
  memset(&own_address_, 0, sizeof(own_address_));
  memset(&other_address_, 0, sizeof(other_address_));
  
  // Fill server information
  own_address_.sin_family = AF_INET;
  own_address_.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
  own_address_.sin_port = htons(OWN_PORT);
  
  // Bind the socket
  int res = bind(sockfd, (const struct sockaddr*)&own_address_, sizeof(own_address_));
  if( res < 0 )
  {
     perror("Bind failed");
     exit(EXIT_FAILURE);
  }
  
  socklen_t len;
  int n;

  other_address_.sin_family = AF_INET;
  other_address_.sin_addr.s_addr = inet_addr("127.0.0.1"); //INADDR_ANY;
  other_address_.sin_port = htons(OTHER_PORT);

  int N = 100;
  for(int i = 0; i < N; ++i)
  {
   struct Data data1;

   data1.age        = 23;
   data1.first_name = strdup("Neil");
   data1.last_name  = strdup("Armstrong");


   unsigned char* buf_with_header = 0;
   size_t buf_with_header_size;  
   buf_with_header_size = serializeWithCommandHeader(&data1, &buf_with_header);

   //struct Data*  data2 = deserializeWithCommandHeader(buf_with_header, buf_with_header_size);
   //printf("After deserializing: data2: age: %d first name: %s last name: %s \n", data2->age, data2->first_name, data2->last_name);    

   //free(data1.first_name);
   //free(data1.last_name);
   //free(data2->first_name);
   //free(data2->last_name);
   //free(data2);

   printf("Sending data to Gateway.... full size: %d \n", buf_with_header_size);
    sendto(sockfd, buf_with_header, buf_with_header_size, 0, (const struct sockaddr *)&other_address_, sizeof(other_address_));
 
    // Receive............
    //n = recvfrom(sockfd, (unsigned char*) buffer, MAXLINE, MSG_DONTWAIT, (struct sockaddr*)NULL, NULL);
    //if(n > 0)
   // {
   //   struct Data* data2  = deserialize(buffer, n, 0);
   //   printf("\t Data 2: Received message with bytes: %d . Age: %d . First name: %s . Last name: %s \n", n, data2->age, data2->first_name, data2->last_name);
   // }
        
    sleep(1);
  }
   
   
}
