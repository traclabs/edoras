#include <conversion_tool/memory_utils.h>
#include <cstring>
#include <stdio.h>
#include <arpa/inet.h>


size_t memcpy_lb_uint32(uint8_t** _buf, const size_t &_offset, const uint32_t &_data)
{
   uint8_t di;
   
   size_t offset = _offset;
   size_t num_bytes = sizeof(_data);
   
   uint32_t data = _data;
   for(size_t i = 0; i < num_bytes; ++i)
   {
      size_t move = 8*(num_bytes - (1 + i));
      di = (data >> move ) & 0xFF;
      memcpy(*_buf + offset, &di, sizeof(uint8_t)); 
      offset += sizeof(uint8_t);       
   }
   
   return sizeof(uint32_t);
}

size_t memcpy_lb_uint16(uint8_t** _buf, const size_t &_offset, const uint16_t &_data)
{
   uint8_t di;
   
   size_t offset = _offset;
   size_t num_bytes = sizeof(_data);
   
   for(size_t i = 0; i < num_bytes; ++i)
   {
      size_t move = 8*(num_bytes - (1 + i));
      di = (_data >> move ) & 0xFF;
      memcpy(*_buf + offset, &di, sizeof(uint8_t)); 
      offset += sizeof(uint8_t);       
   }
   
  return sizeof(uint16_t);
}

size_t memcpy_lb_uint8(uint8_t** _buf, const size_t &_offset, const uint8_t &_data)
{
   uint8_t di;
   
   size_t offset = _offset;
   size_t num_bytes = sizeof(_data);
   
   for(size_t i = 0; i < num_bytes; ++i)
   {
      size_t move = 8*(num_bytes - (1 + i));
      di = (_data >> move ) & 0xFF;
      memcpy(*_buf + offset, &di, sizeof(uint8_t)); 
      offset += sizeof(uint8_t);       
   }
   
  return sizeof(uint8_t);
}

size_t read_lb_uint8(uint8_t* _buf, const size_t &_offset, uint8_t &_data)
{
   memcpy(&_data, _buf + _offset, sizeof(uint8_t)); 
   return sizeof(uint8_t);
}

size_t read_lb_uint16(uint8_t* _buf, const size_t &_offset, uint16_t &_data)
{
   uint16_t data;
   memcpy(&data, _buf + _offset, sizeof(uint16_t));
   _data = ntohs(data);
    
   return sizeof(uint16_t);
}

size_t read_lb_uint32(uint8_t* _buf, const size_t &_offset, uint32_t &_data)
{
   uint32_t data;
   memcpy(&data, _buf + _offset, sizeof(uint32_t)); 
   _data = ntohl(data);
   
   return sizeof(uint32_t);
}

/**
 * @function getBufferString
 */
std::string getBufferString(uint8_t* _buffer, size_t _buffer_size)
{
   if(_buffer == NULL)
     return std::string("");
     
   std::string s = "";
   for(size_t i = 0; i < _buffer_size; i++)
   {   
      char bi[10];
      sprintf(bi, "%02x", *(_buffer + i) );
      s = s +  " " + bi;
      if(i % 8 == 7 )
        s = s + "\n";
   }

   return s;
}


