/**
 * @file parser_utils.h
 */
#include <data_transfer/parser_utils.h>

/**
 * @function from_rcutils_uint_array_to_uint_buffer 
 * @brief
 * @output An array of bytes. It contains:
 * size_t (8 bytes): buffer length
 * size_t (8 bytes): buffer capacity
 * rest of bytes: Serialized data in format used by ROS2 by default (in this case, CDR, from fastrtps)
 * _buffer_size : Parameter that contains the size of the output buffer  = 16 + size_of (rest of bytes)  = 16 + buffer_length
 */
uint8_t* from_rcutils_uint_array_to_uint_buffer(const rcl_serialized_message_t* _serialized_msg, 
                                                size_t &_buffer_size, size_t &_msg_length,
                                                size_t&_msg_capacity)
{
   size_t msg_buffer_length = _serialized_msg->buffer_length;
   size_t msg_buffer_capacity = _serialized_msg->buffer_capacity;


   // Set the buffer_size
   _buffer_size = sizeof(size_t) + sizeof(size_t) + msg_buffer_length;
   
   // Get space for size_t + size_t + msg_buffer
   uint8_t* buffer = new uint8_t[_buffer_size]; 
   
   
   // Fill the space
   uint8_t offset = 0;
   std::memcpy(buffer + offset, &msg_buffer_length, sizeof(size_t));  
   offset += sizeof(msg_buffer_length);
   std::memcpy(buffer + offset, &msg_buffer_capacity, sizeof(size_t));
   offset += sizeof(msg_buffer_capacity);
   std::memcpy(buffer + offset, _serialized_msg->buffer, msg_buffer_length);

   // Return
   
   // DEBUG
   _msg_length = msg_buffer_length;
   _msg_capacity = msg_buffer_capacity;
   
   return buffer;
}


rcutils_uint8_array_t* make_serialized_array(const uint8_t* _buffer)
{
  // Buffer:
  // size_t: (8 bytes) buffer_length
  // size_t: (8 bytes) buffer_capacity
  // Remaining bytes: data
  size_t buffer_length, buffer_capacity;
  
  size_t offset = 0;
  memcpy( &buffer_length, _buffer + offset, sizeof(size_t));
  offset += sizeof(size_t);
  memcpy( &buffer_capacity, _buffer + offset, sizeof(size_t));
  offset += sizeof(size_t);
  
  rcutils_uint8_array_t* serialized_array = new rcutils_uint8_array_t;
  rcutils_allocator_t default_allocator = rcutils_get_default_allocator();
  
  *serialized_array = rcutils_get_zero_initialized_uint8_array();
  rcutils_uint8_array_init(serialized_array, buffer_capacity, &default_allocator);

  // Allocate space for the pointer to the C data
  if( rcutils_uint8_array_resize(serialized_array, buffer_capacity) != RCUTILS_RET_OK)
  {
      printf("edoras_core: Error initializing array for deserialization process. \n");
      return nullptr;
  }
  
   memcpy( (uint8_t*)serialized_array->buffer, _buffer + offset, buffer_length);
   serialized_array->buffer_length = buffer_length;
   
   return serialized_array;
}  

