#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <assert.h>
#include <stdio.h>

#include "serialize_library.h"

size_t serializeWithHeader(const struct Data* data, unsigned char** buf_with_header)
{
   // Serialize the data
   unsigned char* buf     = 0;
   printf("Serializing data.... \n");
   size_t  bufSize = serialize(data, &buf);

   // Add on top
   unsigned char  header[3] = {0xAC, 0x2d, 0x54};
   printf("Printing header values \n");
    for (int i = 0; i < 3; i++) {
        printf("%02x ", header[i]);
    } printf("\n");

   // Imagine you add 3 bytes on top of the serialized data
   size_t headerSize = 3;
   *buf_with_header = calloc(1, bufSize + headerSize);
   printf("* Header size: %d sizeofheader: %d \n", headerSize, sizeof(header));
   size_t offset = 0;
  
   memcpy(*buf_with_header + offset, &header, sizeof(header));  offset += sizeof(header);
   memcpy(*buf_with_header + offset, buf, bufSize);
   
   return bufSize + headerSize;
} 

///////////////
struct Data* deserializeWithHeader(const unsigned char* bufHeader, const size_t bufHeaderSize)
{
  // You know the first 3 bytes are the header
  size_t offset = 0;
  unsigned char header[3];
  memcpy(&header, bufHeader + offset, sizeof(header));
  printf("Deserializing: header: %02x, %02x, %02x \n", header[0], header[1], header[2]);
  offset = offset + sizeof(header);
  struct Data* data = deserialize(bufHeader, bufHeaderSize - sizeof(header), offset);
  return data;
}

int main()
{
   struct Data data1;

   data1.age        = 23;
   data1.first_name = strdup("Neil");
   data1.last_name  = strdup("Armstrong");


   unsigned char* buf_with_header = 0;
   size_t buf_with_header_size;  
   buf_with_header_size = serializeWithHeader(&data1, &buf_with_header);

   struct Data*  data2 = deserializeWithHeader(buf_with_header, buf_with_header_size);
   printf("After deserializing: data2: age: %d first name: %s last name: %s \n", data2->age, data2->first_name, data2->last_name);    

   free(data1.first_name);
   free(data1.last_name);
   //free(data2->first_name);
   //free(data2->last_name);
   //free(data2);

   return 0;
}
